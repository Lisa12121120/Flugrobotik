from .packer import Packer
from .crtp_packer import CrtpPacker

import struct

from typing import Callable


class GenericCommanderPacker(Packer):
    PORT_COMMANDER_GENERIC = 0x07

    SET_SETPOINT_CHANNEL = 0
    META_COMMAND_CHANNEL = 1

    TYPE_STOP = 0
    TYPE_VELOCITY_WORLD = 1
    TYPE_ZDISTANCE = 2
    TYPE_HOVER = 5
    TYPE_FULL_STATE = 6
    TYPE_POSITION = 7

    TYPE_META_COMMAND_NOTIFY_SETPOINTS_STOP = 0

    def __init__(self, crtp_packer_factory: Callable[[int], CrtpPacker]):
        super().__init__(crtp_packer_factory, self.PORT_COMMANDER_GENERIC)

    def send_notify_sendpoints_stop(self, remain_valid_milliseconds):
        data = struct.pack(
            "<BI",
            self.TYPE_META_COMMAND_NOTIFY_SETPOINTS_STOP,
            remain_valid_milliseconds,
        )
        return self._prepare_packet(channel=self.META_COMMAND_CHANNEL, data=data)

    def send_stop_setpoint(self):
        data = struct.pack("<B", self.TYPE_STOP)
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL, data=data)

    def send_velocity_world_setpoint(self, vx, vy, vz, yawrate):
        data = struct.pack("<Bffff", self.TYPE_VELOCITY_WORLD, vx, vy, vz, yawrate)
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL, data=data)

    def send_zdistance_setpoint(self, roll, pitch, yawrate, zdistance):
        data = struct.pack(
            "<Bffff", self.TYPE_ZDISTANCE, roll, pitch, yawrate, zdistance
        )
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL, data=data)

    def send_hover_setpoint(self, vx, vy, yawrate, zdistance):
        data = struct.pack("<Bffff", self.TYPE_HOVER, vx, vy, yawrate, zdistance)
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL, data=data)

    def send_full_state_setpoint(
        self, x, y, z, vx, vy, vz, ax, ay, az, orient_comp, rr, pr, yr
    ):
        data = struct.pack(
            "<BhhhhhhhhhIhhh",
            self.TYPE_FULL_STATE,
            x,
            y,
            z,
            vx,
            vy,
            vz,
            ax,
            ay,
            az,
            orient_comp,
            rr,
            pr,
            yr,
        )
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL, data=data)

    def send_position_setpoint(self, x, y, z, yaw):
        data = struct.pack("<Bffff", self.TYPE_POSITION, x, y, z, yaw)
        return self._prepare_packet(channel=self.SET_SETPOINT_CHANNEL, data=data)
