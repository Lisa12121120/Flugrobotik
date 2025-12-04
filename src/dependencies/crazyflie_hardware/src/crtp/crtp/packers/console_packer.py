from .packer import Packer
from .crtp_packer import CrtpPacker

import struct

from typing import Callable


class ConsolePacker(Packer):
    PORT_CONSOLE = 0

    CHANNEL_CONSOLE = 0

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_CONSOLE)

    def consolepacket(self):
        data = struct.pack("<")
        return self._prepare_packet(channel=self.CHANNEL_CONSOLE, data=data)
