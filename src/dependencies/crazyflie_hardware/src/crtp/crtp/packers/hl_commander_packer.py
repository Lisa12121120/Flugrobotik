from .packer import Packer
from .crtp_packer import CrtpPacker

import struct

from typing import Callable


class HighLevelCommanderPacker(Packer):
    PORT_HL_COMMANDER = 0x08

    CHANNEL_HL_COMMANDER = 0

    COMMAND_SET_GROUP_MASK = 0
    COMMAND_STOP = 3
    COMMAND_GO_TO = 4
    COMMAND_START_TRAJECTORY = 5
    COMMAND_DEFINE_TRAJECTORY = 6
    COMMAND_TAKEOFF_2 = 7
    COMMAND_LAND_2 = 8

    TRAJECTORY_LOCATION_MEM = 1

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_HL_COMMANDER)

    def _prepare_packet(self, data):
        """
        Overwrite, because for HighLevelCommander the channel is always the same
        """
        return super()._prepare_packet(channel=self.CHANNEL_HL_COMMANDER, data=data)

    def set_group_mask(self, group_mask):
        data = struct.pack("<BB", self.COMMAND_SET_GROUP_MASK, group_mask)
        return self._prepare_packet(data)

    def stop(self, group_mask):
        data = struct.pack("<BB", self.COMMAND_STOP, group_mask)
        return self._prepare_packet(data)

    def go_to(self, group_mask, relative, x, y, z, yaw, duration_s):
        data = struct.pack(
            "<BBBfffff",
            self.COMMAND_GO_TO,
            group_mask,
            relative,
            x,
            y,
            z,
            yaw,
            duration_s,
        )
        return self._prepare_packet(data)

    def start_trajectory(
        self, group_mask, relative, reversed, trajectory_id, time_scale
    ):
        data = struct.pack(
            "<BBBBBf",
            self.COMMAND_START_TRAJECTORY,
            group_mask,
            relative,
            reversed,
            trajectory_id,
            time_scale,
        )
        return self._prepare_packet(data)

    def define_trajectory(self, trajectory_id, type, offset, n_pieces):
        data = struct.pack(
            "<BBBBIB",
            self.COMMAND_DEFINE_TRAJECTORY,
            trajectory_id,
            self.TRAJECTORY_LOCATION_MEM,
            type,
            offset,
            n_pieces,
        )
        return self._prepare_packet(data)

    def takeoff(
        self, group_mask, absolute_height_m, target_yaw, use_current_yaw, duration_s
    ):
        data = struct.pack(
            "<BBff?f",
            self.COMMAND_TAKEOFF_2,
            group_mask,
            absolute_height_m,
            target_yaw,
            use_current_yaw,
            duration_s,
        )
        return self._prepare_packet(data)

    def land(
        self, group_mask, absolute_height_m, target_yaw, use_current_yaw, duration_s
    ):
        data = struct.pack(
            "<BBff?f",
            self.COMMAND_LAND_2,
            group_mask,
            absolute_height_m,
            target_yaw,
            use_current_yaw,
            duration_s,
        )
        return self._prepare_packet(data)
