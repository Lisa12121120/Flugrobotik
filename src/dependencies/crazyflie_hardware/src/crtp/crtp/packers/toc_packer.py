from .packer import Packer
from .crtp_packer import CrtpPacker

import struct

from typing import Callable


class TocPacker(Packer):
    TOC_CHANNEL = 0

    # Commands used when accessing the Table of Contents
    CMD_GET_ITEM = 2  # v2; only v2 is supported
    CMD_GET_INFO = 3  # v2; only v2 is supported

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker], port):
        super().__init__(crtp_packer_factory, port)

    def get_toc_info(self):
        data = struct.pack("<B", self.CMD_GET_INFO)
        return self._prepare_packet(channel=self.TOC_CHANNEL, data=data), True, 1

    def get_toc_item(self, index):
        data = struct.pack("<BBB", self.CMD_GET_ITEM, index & 0xFF, (index >> 8) & 0xFF)
        return self._prepare_packet(self.TOC_CHANNEL, data), True, 3
