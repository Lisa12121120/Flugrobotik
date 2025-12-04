from .logic import Logic
from crtp.crtp_link import CrtpLink
from crtp.packers.generic_commander_packer import GenericCommanderPacker
from crtp.packers.crtp_packer import CrtpPacker

from typing import Callable

from crtp.utils.encoding import compress_quaternion


class GenericCommanderLogic(Logic):
    """
    Used for sending control setpoints to the Crazyflie
    """

    def __init__(
        self, crtp_packer_factory: Callable[[int], CrtpPacker], crtp_link: CrtpLink
    ):
        super().__init__(crtp_link)
        self.packer = GenericCommanderPacker(crtp_packer_factory)

    def send_notify_setpoints_stop(self, remain_valid_milliseconds=0):
        """
        Sends a packet so that the priority of the current setpoint to the lowest non-disabled value,
        so any new setpoint regardless of source will overwrite it.
        """
        packet = self.packer.send_notify_sendpoints_stop(remain_valid_milliseconds)
        self.link.send_packet_no_response(packet)

    def send_stop_setpoint(self):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        packet = self.packer.send_stop_setpoint()
        self.link.send_packet_no_response(packet)

    def send_velocity_world_setpoint(self, vx, vy, vz, yawrate):
        """
        Send Velocity in the world frame of reference setpoint with yawrate commands

        vx, vy, vz are in m/s
        yawrate is in degrees/s
        """
        packet = self.packer.send_velocity_world_setpoint(vx, vy, vz, yawrate)
        self.link.send_packet_no_response(packet)

    def send_zdistance_setpoint(self, roll, pitch, yawrate, zdistance):
        """
        Control mode where the height is send as an absolute setpoint (intended
        to be the distance to the surface under the Crazflie), while giving roll,
        pitch and yaw rate commands

        roll, pitch are in degrees
        yawrate is in degrees/s
        zdistance is in meters
        """
        packet = self.packer.send_zdistance_setpoint(roll, pitch, yawrate, zdistance)
        self.link.send_packet_no_response(packet)

    def send_hover_setpoint(self, vx, vy, yawrate, zdistance):
        """
        Control mode where the height is send as an absolute setpoint (intended
        to be the distance to the surface under the Crazflie), while giving x, y velocity
        commands in body-fixed coordinates.

        vx,  vy are in m/s
        yawrate is in degrees/s
        zdistance is in meters
        """
        packet = self.packer.send_hover_setpoint(vx, vy, yawrate, zdistance)
        self.link.send_packet_no_response(packet)

    def send_full_state_setpoint(
        self, pos, vel, acc, orientation, rollrate, pitchrate, yawrate
    ):
        """
        Control mode where the position, velocity, acceleration, orientation and angular
        velocity are sent as absolute (world) values.

        position [x, y, z] are in m
        velocity [vx, vy, vz] are in m/s
        acceleration [ax, ay, az] are in m/s^2
        orientation [qx, qy, qz, qw] are the quaternion components of the orientation
        rollrate, pitchrate, yawrate are in degrees/s
        """

        def vector_to_mm_16bit(vec):
            return int(vec[0] * 1000), int(vec[1] * 1000), int(vec[2] * 1000)

        x, y, z = vector_to_mm_16bit(pos)
        vx, vy, vz = vector_to_mm_16bit(vel)
        ax, ay, az = vector_to_mm_16bit(acc)
        rr, pr, yr = vector_to_mm_16bit([rollrate, pitchrate, yawrate])
        orient_comp = compress_quaternion(orientation)
        packet = self.send_full_state_setpoint(
            x, y, z, vx, vy, vz, ax, ay, az, orient_comp, rr, pr, yr
        )
        self.link.send_packet_no_response(packet)

    def send_position_setpoint(self, x, y, z, yaw):
        """
        Control mode where the position is sent as absolute (world) x,y,z coordinate in
        meter and the yaw is the absolute orientation.

        x, y, z are in m
        yaw is in degrees
        """
        packet = self.packer.send_position_setpoint(x, y, z, yaw)
        self.link.send_packet_no_response(packet)
