from .logic import Logic
from crtp.crtp_link import CrtpLink
from crtp.packers.link_layer_packer import LinkLayerPacker
from crtp.packers.crtp_packer import CrtpPacker

from typing import Callable


class LinkLayerLogic(Logic):

    def __init__(
        self, crtp_packer_factory: Callable[[int], CrtpPacker], crtp_link: CrtpLink
    ):
        super().__init__(crtp_link)
        self.packer = LinkLayerPacker(crtp_packer_factory)

    def send_nullpacket(self):
        packet = self.packer.nullpacket()
        self.link.send_packet_no_response(packet)

    def platform_power_down(self):
        packet = self.packer.platform_power_down()
        self.link.send_packet_no_response(packet)

    def reboot_to_bootloader(self):
        packet = self.packer.reset_init()
        self.link.send_packet_no_response(packet)

        packet = self.packer.reset_to_bootloader()
        self.link.send_packet_no_response(packet)

    def reboot_to_firmware(self):
        packet = self.packer.reset_init()
        self.link.send_packet_no_response(packet)

        packet = self.packer.reset_to_firmware()
        self.link.send_packet_no_response(packet)
