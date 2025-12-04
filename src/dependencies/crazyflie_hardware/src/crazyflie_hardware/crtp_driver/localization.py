import rclpy
from rclpy.node import Node
from rclpy import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .crtp_link_ros import CrtpLinkRos
from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.localization_logic import LocalizationLogic

from broadcaster_interfaces.srv import PosiPoseBroadcastObject

from typing import List


class LocalizationException(Exception):
    pass


class Localization(LocalizationLogic):
    def __init__(self, node: Node, crtp_link: CrtpLinkRos, tf_name: str):
        super().__init__(CrtpPackerRos, crtp_link)
        self.node: Node = node
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.tf_name = tf_name

    #   self.state = 0

    # def start_selflocalization(self, tf_name):
    #    """It is also possible to not use broadcaster for sending external pose.
    #    This does this but is not fully implemented
    #    Args:
    #        tf_name (_type_): _description_
    #    """
    #
    #    self.tf_buffer = Buffer()
    #    self.tf_listener = TransformListener(self.tf_buffer, self.node)
    #    self.timer = self.node.create_timer(
    #        0.2, lambda: self.on_timer(tf_name), callback_group=self.callback_group
    #    )
    #
    # def on_timer(self, tf_name: str):
    #    if self.state == 1:
    #        return
    #    try:
    #        t = self.tf_buffer.lookup_transform("world", tf_name, rclpy.time.Time())
    #    except TransformException as ex:
    #        self.node.get_logger().info("Tracker no Frame")
    #        return
    #
    #    pos = [
    #        t.transform.translation.x * 1,
    #        t.transform.translation.y * 1,
    #        t.transform.translation.z * 1,
    #    ]
    #    self.send_extpos(pos)

    def start_external_tracking(
        self,
        marker_configuration_index: int,
        dynamics_configuration_index: int,
        max_initial_deviation: float,
        initial_position: List[float],
        channel: int,
        datarate: int,
    ) -> bool:
        self.channel: int = channel
        self.datarate: int = datarate

        tracker_success = self.add_to_tracker(
            marker_configuration_index,
            dynamics_configuration_index,
            max_initial_deviation,
            initial_position,
        )
        if not tracker_success:
            raise LocalizationException("Adding to tracker failed.")

        broadcaster_success = self.add_to_broadcaster(channel, datarate)
        if not broadcaster_success:
            self.remove_from_tracker()
            raise LocalizationException("Adding to Broadcaster failed.")

        return True

    def stop_external_tracking(self) -> List:
        tracker_fut = self.remove_from_tracker()
        broadcaster_fut = self.remove_from_broadcaster(self.channel, self.datarate)
        return [tracker_fut, broadcaster_fut]

    def add_to_tracker(
        self,
        marker_configuration_index: int,
        dynamics_configuration_index: int,
        max_initial_deviation: float,
        initial_position: List[float],
    ):
        # Establish Tracking
        from object_tracker_interfaces.srv import AddTrackerObject, RemoveTrackerObject

        add_to_tracker_service = self.node.create_client(
            srv_type=AddTrackerObject,
            srv_name="/tracker/add_object",
            callback_group=self.callback_group,
        )

        if not add_to_tracker_service.wait_for_service(timeout_sec=1.0):
            raise TimeoutError("Add to Tracker Service not available!")

        req = AddTrackerObject.Request()
        req.tf_name.data = self.tf_name
        req.marker_configuration_idx = marker_configuration_index
        req.dynamics_configuration_idx = dynamics_configuration_index
        req.max_initial_deviation = max_initial_deviation
        (
            req.initial_pose.position.x,
            req.initial_pose.position.y,
            req.initial_pose.position.z,
        ) = initial_position
        fut: Future = add_to_tracker_service.call_async(req)
        rclpy.spin_until_future_complete(
            node=self.node, future=fut, executor=self.node.executor, timeout_sec=1.0
        )
        try:
            response: AddTrackerObject.Response = fut.result()
        except:
            raise TimeoutError("Add to Tracker Service did not respond in time.")
        return response.success

    def remove_from_tracker(self):
        from object_tracker_interfaces.srv import RemoveTrackerObject

        remove_from_tracker_service = self.node.create_client(
            srv_type=RemoveTrackerObject,
            srv_name="/tracker/remove_object",
            callback_group=self.callback_group,
        )

        req = RemoveTrackerObject.Request()
        req.tf_name.data = self.tf_name
        return remove_from_tracker_service.call_async(req)

    def add_to_broadcaster(self, channel: int, datarate: int):
        # Establish Broadcasting
        self.add_to_broadcaster_service = self.node.create_client(
            srv_type=PosiPoseBroadcastObject,
            srv_name="/add_posi_pose_object",
            callback_group=self.callback_group,
        )
        if not self.add_to_broadcaster_service.wait_for_service(timeout_sec=1.0):
            raise TimeoutError("Add to Broadcaster Service not available!")

        req = PosiPoseBroadcastObject.Request()
        req.channel = channel
        req.data_rate = datarate
        req.tf_frame_id = self.tf_name
        fut: Future = self.add_to_broadcaster_service.call_async(req)
        rclpy.spin_until_future_complete(
            node=self.node, future=fut, executor=self.node.executor, timeout_sec=1.0
        )
        try:
            response: PosiPoseBroadcastObject.Response = fut.result()
        except:
            raise TimeoutError("Add to Broadcaster Service did not respond.")
        return response.success

    def remove_from_broadcaster(self, channel: int, datarate: int):
        self.remove_from_broadcaster_service = self.node.create_client(
            srv_type=PosiPoseBroadcastObject,
            srv_name="/remove_posi_pose_object",
            callback_group=self.callback_group,
        )
        if self.remove_from_broadcaster_service.wait_for_service(timeout_sec=0.5):
            req = PosiPoseBroadcastObject.Request()
            req.channel = channel
            req.data_rate = datarate
            req.tf_frame_id = self.tf_name
            return self.remove_from_broadcaster_service.call_async(req)
        return None
