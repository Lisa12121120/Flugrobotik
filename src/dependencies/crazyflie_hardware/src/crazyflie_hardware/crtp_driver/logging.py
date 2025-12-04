from typing import List
from crazyflie_interfaces_python.server.logblock import LogBlockServer
from rclpy.node import Node
from .crtp_link_ros import CrtpLinkRos

from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.logging_logic import LoggingLogic
from crazyflie_interfaces_python.server import LoggingServer
import os
from crtp_interfaces.msg import CrtpPacket


from typing import List, Dict, Iterator

import struct


class Logging(LoggingServer, LoggingLogic):
    def __init__(self, node: Node, crtp_link: CrtpLinkRos):
        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "log")
        LoggingLogic.__init__(self, CrtpPackerRos, crtp_link, path)
        LoggingServer.__init__(self, node)
        crtp_link.add_callback(5, self.crtp_callback)

        self._next_id: Iterator = iter(range(256))
        self.block_servers: Dict[int, LogBlockServer] = {}
        self.initialize_toc()

    def crtp_callback(self, msg: CrtpPacket):
        if msg.channel == 2:
            block_id, ts1, ts2, ts3 = struct.unpack("<BBBB", bytearray(msg.data[:4]))
            values = [
                float(val)
                for val in self.unpack_block(block_id, bytearray(msg.data[4:]))
            ]

            if block_id in self.block_servers.keys():
                self.block_servers[block_id].send_data(values)

    # Override
    def download_toc(self) -> None:
        self.send_download_toc_items()

    # Override
    def get_toc_info(self) -> None:
        nbr_of_items, crc = self.send_get_toc_info()
        self.node.get_logger().info(str("NBR of Items: " + str(nbr_of_items)))

    # Override
    def create_log_block(self, variables: List[str], log_block: LogBlockServer) -> None:
        log_block_id = self.next_id
        self.add_block(log_block_id, variables)

        log_block.set_log_block_start_callback(
            lambda period_ms, log_block_id=log_block_id: self.start_block(
                log_block_id, period_ms
            )
        )

        log_block.set_log_block_stop_callback(
            lambda log_block_id=log_block_id: self.stop_block(log_block_id)
        )
        self.block_servers[log_block_id] = log_block

    @property
    def next_id(self) -> int:
        return next(self._next_id)
