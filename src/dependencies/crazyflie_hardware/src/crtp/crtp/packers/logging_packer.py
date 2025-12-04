from .toc_packer import TocPacker
from .crtp_packer import CrtpPacker

import struct

from typing import Callable


class LoggingPacker(TocPacker):
    PORT_LOGGING = 5

    # Channels used for the logging port
    TOC_CHANNEL = 0
    CONTROL_CHANNEL = 1
    LOGDATA_CHANNEL = 2

    # Commands used when accessing the Log configurations
    CMD_CREATE_BLOCK = 0
    CMD_APPEND_BLOCK = 1
    CMD_DELETE_BLOCK = 2
    CMD_START_LOGGING = 3
    CMD_STOP_LOGGING = 4
    CMD_RESET_LOGGING = 5
    CMD_CREATE_BLOCK_V2 = 6
    CMD_APPEND_BLOCK_V2 = 7

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_LOGGING)

    def _create_block_content(self, content):
        data = struct.pack("<")
        for el in content:
            storage_and_fetch, index = el
            data += struct.pack(
                "<BBB", storage_and_fetch, index & 0xFF, (index >> 8) & 0xFF
            )  # storage and fetch byte
        return data

    def create_block(self, index, content):
        data = struct.pack("<BB", self.CMD_CREATE_BLOCK_V2, index)
        data += self._create_block_content(content)
        return self._prepare_packet(self.CONTROL_CHANNEL, data), True, 2

    def start_block(self, index, period):
        data = struct.pack("<BBB", self.CMD_START_LOGGING, index, period)
        return self._prepare_packet(self.CONTROL_CHANNEL, data), True, 2

    def stop_block(self, index):
        data = struct.pack("<BB", self.CMD_STOP_LOGGING, index)
        return self._prepare_packet(self.CONTROL_CHANNEL, data), True, 2
