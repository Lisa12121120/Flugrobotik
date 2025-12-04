from abc import ABC

from crtp.crtp_link import CrtpLink


class Logic(ABC):
    def __init__(self, crtp_link: CrtpLink):
        self.link: CrtpLink = crtp_link
