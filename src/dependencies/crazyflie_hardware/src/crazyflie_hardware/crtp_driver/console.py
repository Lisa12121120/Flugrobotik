from rclpy.node import Node

from crtp_driver.crtp_link_ros import CrtpLinkRos
from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.console_logic import ConsoleLogic
from crazyflie_interfaces_python.server import ConsoleServer
from crtp_interfaces.msg import CrtpPacket


class Console(ConsoleServer, ConsoleLogic):
    def __init__(self, node: Node, crtp_link: CrtpLinkRos):
        self.node = node
        ConsoleLogic.__init__(self, CrtpPackerRos, crtp_link)
        ConsoleServer.__init__(self, node)
        crtp_link.add_callback(0, self.crtp_callback)

        self._message: str = ""  # Message can be from multiple consecutive packets

    def crtp_callback(self, packet: CrtpPacket):
        if packet.channel == 0:  # This is a text message
            data = packet.data
            data_length = packet.data_length
            for i in range(data_length):
                self._message += chr(data[i])

            if self._message.endswith("\n"):
                self.console_message(self._message[:-1])
                self.node.get_logger().debug(self._message[:-1])
                self._message = ""
