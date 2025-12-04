from .logic import Logic
from crtp.crtp_link import CrtpLink
from .toc.toc import Toc
from .toc.toccache import TocCache
from crtp.packers.toc_packer import TocPacker

import struct


class TocLogic(Logic):

    def __init__(
        self,
        packer: TocPacker,  # crtp_packer_factory: Callable[[int], CrtpPacker],
        crtp_link: CrtpLink,
        ElementType,
        path: str,
    ):
        super().__init__(crtp_link)
        self.packer: TocPacker = packer
        self.ElementType = ElementType

        self.toc = Toc()
        self.toc_cache = TocCache(rw_cache=path)

        self.nbr_of_items = None
        self.crc = None

    def _to_toc_item(self, data):
        ident = struct.unpack("<H", data[1:3])[0]
        data = bytearray(data[3:])
        element = self.ElementType(ident, data)
        self.toc.add_element(element)

    def initialize_toc(self):
        nbr_of_items, crc = self.send_get_toc_info()
        cache_data = self.toc_cache.fetch(crc)
        if cache_data:
            self.toc.toc = cache_data
        else:
            self.send_download_toc_items()

    def send_get_toc_info(self):
        packet, expects_response, matching_bytes = self.packer.get_toc_info()
        resp_data = self.link.send_packet(packet, expects_response, matching_bytes).data
        [self.nbr_of_items, self.crc] = struct.unpack("<HI", resp_data[1:7])
        return self.nbr_of_items, self.crc

    def send_download_toc_items(self):
        if self.nbr_of_items == None or self.crc == None:
            self.send_get_toc_info()
        packets = []
        for i in range(self.nbr_of_items):
            packets.append(self.packer.get_toc_item(i))

        responses = self.link.send_batch_request(packets)
        for result in responses:
            self._to_toc_item(result.packet.data[: result.packet.data_length])

        self.toc_cache.insert(self.crc, self.toc.toc)
