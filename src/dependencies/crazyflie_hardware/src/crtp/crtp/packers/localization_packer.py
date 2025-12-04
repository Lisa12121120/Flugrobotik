from .packer import Packer
from .crtp_packer import CrtpPacker

import struct

from typing import Callable


class LocalizationPacker(Packer):
    PORT_LOCALIZATION = 6

    # Implemented channels
    POSITION_CH = 0
    GENERIC_CH = 1

    # Location message types for generig channel
    RANGE_STREAM_REPORT = 0
    RANGE_STREAM_REPORT_FP16 = 1
    LPS_SHORT_LPP_PACKET = 2
    EMERGENCY_STOP = 3
    EMERGENCY_STOP_WATCHDOG = 4
    COMM_GNSS_NMEA = 6
    COMM_GNSS_PROPRIETARY = 7
    EXT_POSE = 8
    EXT_POSE_PACKED = 9
    LH_ANGLE_STREAM = 10
    LH_PERSIST_DATA = 11

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_LOCALIZATION)

    def send_extpos(self, pos):
        data = struct.pack("<fff", pos[0], pos[1], pos[2])
        return self._prepare_packet(channel=self.POSITION_CH, data=data)

    def send_short_lpp_packet(self, dest_id, lpp_data):
        data = struct.pack("<BB", self.LPS_SHORT_LPP_PACKET, dest_id)
        data += lpp_data
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)

    def send_emergency_stop(self):
        data = struct.pack("<B", self.EMERGENCY_STOP)
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)

    def send_emergency_stop_watchdog(self):
        data = struct.pack("<B", self.EMERGENCY_STOP_WATCHDOG)
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)

    def send_extpose(self, pos, quat):
        data = struct.pack(
            "<Bfffffff",
            self.EXT_POSE,
            pos[0],
            pos[1],
            pos[2],
            quat[0],
            quat[1],
            quat[2],
            quat[3],
        )
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)

    def send_lh_persist_data_packet(self, mask_geo, mask_calib):
        data = struct.pack("<BHH", self.LH_PERSIST_DATA, mask_geo, mask_calib)
        return self._prepare_packet(channel=self.GENERIC_CH, data=data)
