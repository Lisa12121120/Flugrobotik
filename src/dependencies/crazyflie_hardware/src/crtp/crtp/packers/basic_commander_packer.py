from .packer import Packer
from .crtp_packer import CrtpPacker

import struct

from typing import Callable


class BasicCommanderPacker(Packer):
    PORT_BASIC_COMMANDER = 0x03

    CHANNEL = 0x0

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_BASIC_COMMANDER)

    def send_setpoint(self, roll, pitch, yawrate, thrust):
        data = struct.pack("<fffH", roll, pitch, yawrate, thrust)
        return self._prepare_packet(data=data)
