from abc import ABC, abstractmethod


class CrtpPacker(ABC):
    def __init__(self, port: int):
        self.port: int = port

    @abstractmethod
    def prepare_packet(self, channel: int, data: bytes):
        return NotImplementedError("No valid CrtpPacker implementation")
