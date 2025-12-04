from crtp.packers.crtp_packer import CrtpPacker
from crtp.crtp_link import CrtpLink
from crtp.packers.logging_packer import LoggingPacker

from .toc.logging import LogTocElement
from .toc_logic import TocLogic

from typing import Callable, List, Tuple, Dict

import struct


class LoggingLogic(TocLogic):

    def __init__(
        self,
        crtp_packer_factory: Callable[[int], CrtpPacker],
        crtp_link: CrtpLink,
        path: str,
    ):
        self.__packer = LoggingPacker(crtp_packer_factory)
        super().__init__(self.__packer, crtp_link, LogTocElement, path)

        self.blocks: Dict[int, Tuple[str, int]] = (
            {}
        )  # the string is bytemask for unpacking, the int the length

    def unpack_block(self, block_id: int, data: bytearray) -> List[float]:
        if block_id not in self.blocks.keys():
            return []
        unpack_string, length = self.blocks[block_id]
        return struct.unpack(unpack_string, data[:length])

    def start_block(self, id: int, period_ms_d10: int):
        # The period in the packet is divided by 10 because for transfer only one byte is used
        packet, expects_response, matching = self.__packer.start_block(
            id, period_ms_d10
        )
        self.link.send_packet_no_response(packet)

    def stop_block(self, id: int):
        packet, expects_response, matching = self.__packer.stop_block(id)
        self.link.send_packet_no_response(packet)

    def add_block(self, id: int, variables: List[str]):
        elements: List[Tuple] = []
        unpack_string = "<"
        total_bytelength = 0
        for variable_name in variables:
            element = self.toc.get_element_by_complete_name(variable_name)
            type_id = LogTocElement.get_id_from_cstring(element.ctype)
            unpack_string += LogTocElement.get_unpack_string_from_id(type_id).strip(
                "<"
            )  # only add the key not the >
            total_bytelength += LogTocElement.get_size_from_id(type_id)
            elements.append((type_id, element.ident))

        self.blocks[id] = (unpack_string, total_bytelength)

        packet, expects_response, matching = self.__packer.create_block(id, elements)
        self.link.send_packet_no_response(packet)
