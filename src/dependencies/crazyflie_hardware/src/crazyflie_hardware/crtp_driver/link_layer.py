from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .crtp_link_ros import CrtpLinkRos
from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.link_layer_logic import LinkLayerLogic

from std_msgs.msg import Empty


class LinkLayer(LinkLayerLogic):
    def __init__(self, node: Node, crtp_link: CrtpLinkRos):
        super().__init__(CrtpPackerRos, crtp_link)

        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(
            Empty,
            "~/platform_power_down",
            self._platform_power_down,
            10,
            callback_group=callback_group,
        )
        node.create_subscription(
            Empty,
            "~/reboot_to_firmware",
            self._reboot_to_firmware,
            10,
            callback_group=callback_group,
        )
        node.create_subscription(
            Empty,
            "~/reboot_to_bootloader",
            self._reboot_to_bootloader,
            10,
            callback_group=callback_group,
        )

        node.create_subscription(
            Empty,
            "~/send_nullpacket",
            self._send_nullpacket,
            10,
            callback_group=callback_group,
        )

    def _send_nullpacket(self, msg: Empty):
        self.send_nullpacket()

    def _platform_power_down(self, msg: Empty):
        self.platform_power_down()

    def _reboot_to_firmware(self, msg: Empty):
        self.reboot_to_firmware()

    def _reboot_to_bootloader(self, msg: Empty):
        self.reboot_to_bootloader()
