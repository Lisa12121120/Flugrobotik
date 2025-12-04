import numpy as np
from crtp_interfaces.msg import CrtpPacket

from crtp.packers.crtp_packer import CrtpPacker


class CrtpPackerRos(CrtpPacker):
    def __init__(self, port: int):
        super().__init__(port)

    def _struct_to_data(self, data: bytes) -> np.ndarray:
        """the data field must be an array of size 31"""
        np_array = np.array([p for p in data], dtype=np.uint8)
        return np.pad(
            np_array, (0, len(CrtpPacket().data) - len(data)), mode="constant"
        )

    def _fill_packet_data(self, pk: CrtpPacket, data: bytes) -> None:
        pk.data = self._struct_to_data(data)
        pk.data_length = len(data)

    # Overrride
    def prepare_packet(self, channel: int, data: bytes) -> CrtpPacket:
        pk = CrtpPacket()
        pk.port = self.port
        pk.channel = channel
        self._fill_packet_data(pk, data)
        pk.data_length = len(data)

        return pk
