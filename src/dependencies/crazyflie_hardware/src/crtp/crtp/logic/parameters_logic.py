from crtp.packers.parameters_packer import ParametersPacker
from crtp.crtp_link import CrtpLink
from crtp.packers.crtp_packer import CrtpPacker
from .toc.parameters import ParamTocElement
from .toc_logic import TocLogic

from typing import Callable
from numbers import Number


class ParametersLogic(TocLogic):

    def __init__(
        self,
        crtp_packer_factory: Callable[[int], CrtpPacker],
        crtp_link: CrtpLink,
        path: str,
    ):
        self.__packer = ParametersPacker(crtp_packer_factory)
        super().__init__(self.__packer, crtp_link, ParamTocElement, path)

    def send_set_parameter(self, group: str, name: str, value: Number):
        toc_element = self.toc.get_element(group, name)  ## Error checking!!
        id = toc_element.ident
        if toc_element.pytype == "<f" or toc_element.pytype == "<d":
            value_nr = float(value)
        else:
            value_nr = int(value)

        packet = self.__packer.set_parameter(id, toc_element.pytype, value_nr)
        self.link.send_packet_no_response(packet)
