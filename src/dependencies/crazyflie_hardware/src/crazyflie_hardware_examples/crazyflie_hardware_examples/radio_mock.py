#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import struct
from crtp_interfaces.srv import CrtpPacketSend


class RadioMock(Node):
    def __init__(self):
        super().__init__("radio_mock")
        self.create_service(
            CrtpPacketSend, "crazyradio/send_crtp_packet", self.handle_response
        )

    def __del__(self):
        self.destroy_node()

    def handle_response(
        self, req: CrtpPacketSend.Request, resp: CrtpPacketSend.Response
    ):
        link = req.link
        pkt = req.packet
        string = f"{link.channel},{link.address}-{pkt.port}:{pkt.channel}-{pkt.data}"
        self.get_logger().info(string)

        return resp


def main():
    rclpy.init()
    listener = RadioMock()

    try:
        while rclpy.ok():
            rclpy.spin_once(listener)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        exit()


if __name__ == "__main__":
    main()
