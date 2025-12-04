from .crtp_packer import CrtpPacker

from typing import Callable


class Packer:
    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker], port: int):
        self.packer: CrtpPacker = crtp_packer_factory(port)

    def _prepare_packet(self, channel: int, data: bytes):
        return self.packer.prepare_packet(channel=channel, data=data)
