from .logic import Logic
from crtp.crtp_link import CrtpLink
from crtp.packers.console_packer import ConsolePacker
from crtp.packers.crtp_packer import CrtpPacker

from typing import Callable


class ConsoleLogic(Logic):

    def __init__(
        self, crtp_packer_factory: Callable[[int], CrtpPacker], crtp_link: CrtpLink
    ):
        super().__init__(crtp_link)
        self.packer = ConsolePacker(crtp_packer_factory)

    def send_consolepacket(self):
        packet = self.packer.consolepacket()
        self.link.send_packet_no_response(packet)
