import struct
from .logic import Logic
from crtp.crtp_link import CrtpLink
from crtp.packers.localization_packer import LocalizationPacker
from crtp.packers.crtp_packer import CrtpPacker

from crtp.utils.encoding import fp16_to_float

from typing import Callable


class LocalizationLogic(Logic):
    """
    Handle localization-related data communication with the Crazyflie
    """

    def __init__(
        self, crtp_packer_factory: Callable[[int], CrtpPacker], crtp_link: CrtpLink
    ):
        super().__init__(crtp_link)
        self.packer = LocalizationPacker(crtp_packer_factory)

    def send_extpos(self, pos):
        """
        Send the current Crazyflie X, Y, Z position. This is going to be
        forwarded to the Crazyflie's position estimator.
        """
        packet = self.packer.send_extpos(pos)
        self.link.send_packet_no_response(packet)

    def send_short_lpp_packet(self, dest_id, data):
        """
        Send ultra-wide-band LPP packet to dest_id
        """
        packet = self.packer.send_short_lpp_packet(dest_id, data)
        self.link.send_packet_no_response(packet)

    def send_emergency_stop(self):
        """
        Send emergency stop
        """
        packet = self.packer.send_emergency_stop()
        self.link.send_packet_no_response(packet)

    def send_emergency_stop_watchdog(self):
        """
        Send emergency stop watchdog
        """
        packet = self.packer.send_emergency_stop_watchdog()
        self.link.send_packet_no_response(packet)

    def send_extpose(self, pos, quat):
        """
        Send the current Crazyflie pose (position [x, y, z] and
        attitude quaternion [qx, qy, qz, qw]). This is going to be forwarded
        to the Crazyflie's position estimator.
        """
        packet = self.packer.send_extpose(pos, quat)
        self.link.send_packet_no_response(packet)

    def send_lh_persist_data_packet(self, geo_list, calib_list):
        """
        Send geometry and calibration data to persistent memory subsystem
        """
        geo_list.sort()
        calib_list.sort()
        max_bs_nr = 15
        if len(geo_list) > 0:
            if geo_list[0] < 0 or geo_list[-1] > max_bs_nr:
                raise Exception("Geometry BS list is not valid")
        if len(calib_list) > 0:
            if calib_list[0] < 0 or calib_list[-1] > max_bs_nr:
                raise Exception("Calibration BS list is not valid")

        mask_geo = 0
        mask_calib = 0
        for bs in geo_list:
            mask_geo += 1 << bs
        for bs in calib_list:
            mask_calib += 1 << bs

        packet = self.packer.send_lh_persist_data_packet(mask_geo, mask_calib)
        self.link.send_packet_no_response(packet)

    def _decode_lh_angle(self, data):
        decoded_data = {}

        raw_data = struct.unpack("<Bfhhhfhhh", data)

        decoded_data["basestation"] = raw_data[0]
        decoded_data["x"] = [0, 0, 0, 0]
        decoded_data["x"][0] = raw_data[1]
        decoded_data["x"][1] = raw_data[1] - fp16_to_float(raw_data[2])
        decoded_data["x"][2] = raw_data[1] - fp16_to_float(raw_data[3])
        decoded_data["x"][3] = raw_data[1] - fp16_to_float(raw_data[4])
        decoded_data["y"] = [0, 0, 0, 0]
        decoded_data["y"][0] = raw_data[5]
        decoded_data["y"][1] = raw_data[5] - fp16_to_float(raw_data[6])
        decoded_data["y"][2] = raw_data[5] - fp16_to_float(raw_data[7])
        decoded_data["y"][3] = raw_data[5] - fp16_to_float(raw_data[8])

        return decoded_data
