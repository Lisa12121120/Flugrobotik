from .packer import Packer
from .crtp_packer import CrtpPacker

import struct

from typing import Callable


class LinkLayerPacker(Packer):
    PORT_LINK = 0xF

    CHANNEL_ECHO = 0
    CHANNEL_SOURCE = 1
    CHANNEL_SINK = 2
    CHANNEL_NULL = 3

    CHANNEL_LINK = 0xF

    BOOTLOADER_HEADER = 0xFE
    BOOTLOADER_CMD_ALLOFF = 0x01
    BOOTLOADER_CMD_SYSOFF = 0x02
    BOOTLOADER_CMD_SYSON = 0x03
    BOOTLOADER_CMD_GETVBAT = 0x04
    BOOTLOADER_CMD_RESET_INIT = 0xFF
    BOOTLOADER_CMD_RESET = 0xF0

    BOOTLOADER_RESET_TO_BOOTLOADER = 0
    BOOTLOADER_RESET_TO_FIRMWARE = 1

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_LINK)

    def echopacket(self):
        data = struct.pack("<")
        return self._prepare_packet(channel=self.CHANNEL_ECHO, data=data)

    def sourcepacket(self):
        data = struct.pack("<")
        return self._prepare_packet(channel=self.CHANNEL_SOURCE, data=data)

    def sinkpacket(self):
        data = struct.pack("<")
        return self._prepare_packet(channel=self.CHANNEL_SINK, data=data)

    def nullpacket(self):
        data = struct.pack("<")
        return self._prepare_packet(channel=self.CHANNEL_NULL, data=data)

    def get_vbat(self):
        data = struct.pack("<BB", self.BOOTLOADER_HEADER, self.BOOTLOADER_CMD_GETVBAT)
        return self._prepare_packet(channel=self.CHANNEL_LINK, data=data)

    def platform_power_down(self):
        data = struct.pack("<BB", self.BOOTLOADER_HEADER, self.BOOTLOADER_CMD_ALLOFF)
        return self._prepare_packet(channel=self.CHANNEL_LINK, data=data)

    def stm_power_down(self):
        data = struct.pack("<BB", self.BOOTLOADER_HEADER, self.BOOTLOADER_CMD_SYSOFF)
        return self._prepare_packet(channel=self.CHANNEL_LINK, data=data)

    def stm_power_up(self):
        data = struct.pack("<BB", self.BOOTLOADER_HEADER, self.BOOTLOADER_CMD_SYSON)
        return self._prepare_packet(channel=self.CHANNEL_LINK, data=data)

    def reset_init(self):
        data = struct.pack(
            "<BB", self.BOOTLOADER_HEADER, self.BOOTLOADER_CMD_RESET_INIT
        )
        return self._prepare_packet(channel=self.CHANNEL_LINK, data=data)

    def reset_to_bootloader(self):
        """
        Resets the cf to bootloader, reset_init has to be called before this
        """
        data = struct.pack(
            "<BBB",
            self.BOOTLOADER_HEADER,
            self.BOOTLOADER_CMD_RESET,
            self.BOOTLOADER_RESET_TO_BOOTLOADER,
        )
        return self._prepare_packet(channel=self.CHANNEL_LINK, data=data)

    def reset_to_firmware(self):
        """
        Resets the cf to firmware, reset_init has to be called before this
        """
        data = struct.pack(
            "<BBB",
            self.BOOTLOADER_HEADER,
            self.BOOTLOADER_CMD_RESET,
            self.BOOTLOADER_RESET_TO_FIRMWARE,
        )
        return self._prepare_packet(channel=self.CHANNEL_LINK, data=data)
