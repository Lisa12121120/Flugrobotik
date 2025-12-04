from abc import ABC, abstractmethod

from typing import Tuple, Any, List


class CrtpLink(ABC):
    def __init__(self, channel: int, address: Tuple, datarate: int):
        self.channel: int = channel
        self.address: Tuple[int] = address
        self.datarate: int = datarate

    @abstractmethod
    def send_packet_no_response(
        self, packet, expects_response: bool = False, matching_bytes: int = 0
    ) -> Any:
        return NotImplementedError(
            "Missing send packet no response implementation in Link!"
        )

    @abstractmethod
    def send_packet(self, packet, expects_response: bool, matching_bytes: int) -> Any:
        return NotImplementedError("Missing send packet implementation in Link")

    @abstractmethod
    def send_batch_request(self, packets: List[Any]) -> List[Any]:
        return NotImplementedError("Missing send batch request implementation in Link!")
