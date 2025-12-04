#!/usr/bin/env python3

import math
import struct
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration

from broadcaster_interfaces.srv import PosiPoseBroadcastObject
from tf2_ros import TransformException
from crazyflie_interfaces_python.positions import CfPositionBuffer

from crtp_driver.crtp_packer_ros import CrtpPackerRos
from crtp_driver.crtp_link_ros import CrtpLinkRos
from crtp_interfaces.msg import CrtpPacket

from crtp.packers.packer import Packer
from crtp.packers.crtp_packer import CrtpPacker

from crtp.logic.logic import Logic

from typing import Callable

### This needs to be ported to crazyflie_interfaces_python
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
from crazyflie_interfaces.msg import PoseStampedArray
from geometry_msgs.msg import PoseStamped

from typing import Dict

class CfPositionListener:
    def __init__(self, buffer: CfPositionBuffer, node: Node):
        self._buffer = buffer
        self._node = node

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._cf_position_sub = node.create_subscription(
            msg_type=PoseStampedArray,
            topic="/cf_positions",
            callback=self.callback,
            qos_profile=qos,
        )

    def __del__(self) -> None:
        self.unregister()

    def unregister(self): 
        self._node.destroy_subscription(self._cf_position_sub)

    def callback(self, msg: PoseStampedArray):
        pose_stamped: PoseStamped
        for pose_stamped in msg.poses:
            self._buffer.set_position(pose_stamped, msg.header.frame_id)


class Broadcaster(Node):
    def __init__(self):
        super().__init__("broadcaster")
        self.world = (
            self.declare_parameter("world", "world").get_parameter_value().string_value
        )
        self.position_only = True  # self.declare_parameter('position_only', True).get_parameter_value().bool_value
        self.hz = self.declare_parameter("hz", 10).get_parameter_value().integer_value

        self.cf_buffer = CfPositionBuffer(self)
        self.cf_listener = CfPositionListener(self.cf_buffer, self)

        self.timer = self.create_timer(1.0 / self.hz, self.run, callback_group=MutuallyExclusiveCallbackGroup())
        # https://github.com/USC-ACTLab/crazyswarm/blob/master/ros_ws/src/crazyswarm/src/crazyswarm_server.cpp 810 Paar infos über broadcasting Adresse
        # https://github.com/whoenig/crazyflie_cpp/blob/25bc72c120f8cea6664dd24e334eefeb7c9606ca/src/Crazyflie.cpp alter code von broadcaster

        self.broadcaster_commander = PositionBroadcasterCommander(self)

    def run(self):
        id_positions_by_channel: Dict[int, "list[int, list[float]]"] = {} # for each channel list of ids and positions
        for position_ in list(self._get_external_positions()):
            id, position, channel = position_
            if channel not in id_positions_by_channel.keys():
                id_positions_by_channel[channel] = []
            id_positions_by_channel[channel].append((id, position)) 
        

        for channel in id_positions_by_channel.keys():
            self.broadcaster_commander.send_external_positions(id_positions_by_channel[channel], channel)

    def _get_external_positions(self) -> "tuple[int, list[float], int]": # representing id, pos, channel
        for frame in self.broadcaster_commander.frame_channels.keys():
            pose_stamped = self.cf_buffer.get_position(frame)
            if pose_stamped is None:
                continue
            id_ = self._get_id_of_frame(frame)
            pos = [
                pose_stamped.pose.position.x * 1000,
                pose_stamped.pose.position.y * 1000,
                pose_stamped.pose.position.z * 1000,
            ]
            yield id_, pos, self.broadcaster_commander.frame_channels[frame][0]
            # somehow frame_channel is a list of channels. 

    def _get_id_of_frame(self, frame) -> int:
        id_ = frame.replace("cf", "")
        if id_.isdigit():
            return int(id_)
        return 0  # TODO check interactables in crazyswarm for if of non cf frames


class BroadcasterLogic:
    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker], node: Node):
        self.node = node
        self.packer = BroadcasterPacker(crtp_packer_factory)
        self.address = (0xFF, 0xE7, 0xE7, 0xE7, 0xE7)

        self.frame_channels = {}  # to keep track of all frames and their channels
        self.crtp_links = []

    def add_object(self, channel: int, frame: str, data_rate: int = 2) -> bool:
        if channel != 0:
            if frame not in self.frame_channels.keys():
                # multiple channels can use the same id
                self.frame_channels[frame] = [channel]
            # Add a link if there is no link for this channel
            if not channel in (link.channel for link in self.crtp_links):
                self.node.get_logger().debug(f"Adding channel {channel}")
                self.crtp_links.append(
                    CrtpLinkRos(self.node, channel, self.address, data_rate, None)
                )
        return True

    def remove_object(self, channel: int, frame: str) -> bool:
        if frame in self.frame_channels.keys():
            if frame in self.frame_channels.keys():
                if len(self.frame_channels[frame]) == 1:
                    self.frame_channels.pop(frame)
                else:
                    self.frame_channels[frame].remove(channel)

                # remove link if no more frames are using it
                if not any(channel in val for val in self.frame_channels.values()):
                    self.crtp_links = [
                        link for link in self.crtp_links if link.channel != channel
                    ]
            return True
        return False

    def send_external_positions(self, id_positions, channel):
        for i in range(math.ceil(len(id_positions) / 4)):
            packets = []
            for id_, pos in id_positions[i * 4 : i * 4 + 4]:
                # self.node.get_logger().info(f"Sending position for {id_}: {pos}")
                packets.append(
                    self.packer.create_external_position_packet(
                        id_, int(pos[0]), int(pos[1]), int(pos[2])
                    )
                )
            packet = self.packer.send_external_positions(packets)

            # send packet to all channels
            for link in self.crtp_links:
                if link.channel == channel:
                    link.send_packet_no_response(packet)
            # TODO send2packets in https://github.com/whoenig/crazyflie_cpp/blob/25bc72c120f8cea6664dd24e334eefeb7c9606ca/src/Crazyflie.cpp


class PositionBroadcasterCommander(BroadcasterLogic):
    def __init__(self, node: Node):
        super().__init__(CrtpPackerRos, node)
        self.node = node
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.add_service = node.create_service(
            PosiPoseBroadcastObject, "add_posi_pose_object", self._add_object, callback_group=self.callback_group
        )
        self.remove_service = node.create_service(
            PosiPoseBroadcastObject, "remove_posi_pose_object", self._remove_object, callback_group=self.callback_group
        )

    def _add_object(self, request, response):
        response.success = self.add_object(
            request.channel, request.tf_frame_id, request.data_rate
        )
        return response

    def _remove_object(self, request, response):
        response.success = self.remove_object(request.channel, request.tf_frame_id)
        return response


class BroadcasterPacker(Packer):

    PORT_LOCALIZATION = 6
    POSITION_CH = 2

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_LOCALIZATION)

    def send_external_positions(self, position_packets: list) -> CrtpPacket:
        data = b""
        for packet in position_packets[:4]:
            data += packet
        return self._prepare_packet(channel=self.POSITION_CH, data=data)

    def create_external_position_packet(
        self, id_: int, x: int, y: int, z: int
    ) -> bytes:
        return struct.pack("<Bhhh", id_, x, y, z)


def main(args=None):
    rclpy.init(args=args)
    bc = Broadcaster()
    executor = MultiThreadedExecutor()
    executor.add_node(bc)
    try:
        executor.spin()
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        exit()


if __name__ == "__main__":
    main()
