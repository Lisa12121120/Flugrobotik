from .logic import Logic
from crtp.crtp_link import CrtpLink
from crtp.packers.hl_commander_packer import HighLevelCommanderPacker
from crtp.packers.crtp_packer import CrtpPacker

from typing import Callable


class HighLevelCommanderLogic(Logic):
    ALL_GROUPS = 0

    TRAJECTORY_TYPE_POLY4D = 0
    TRAJECTORY_TYPE_POLY4D_COMPRESSED = 1

    def __init__(
        self, crtp_packer_factory: Callable[[int], CrtpPacker], crtp_link: CrtpLink
    ):
        super().__init__(crtp_link)
        self.packer = HighLevelCommanderPacker(crtp_packer_factory)

    def send_set_group_mask(self, group_mask=ALL_GROUPS):
        """
        Set the group mask that the Crazyflie belongs to

        :param group_mask: Mask for which groups this CF belongs to
        """
        packet = self.packer.set_group_mask(group_mask)
        self.link.send_packet_no_response(packet)

    def send_stop(self, group_mask=ALL_GROUPS):
        """
        stops the current trajectory (turns off the motors)

        :param group_mask: Mask for which CFs this should apply to
        :return:
        """
        packet = self.packer.stop(group_mask)
        self.link.send_packet_no_response(packet)

    def send_go_to(
        self, x, y, z, yaw, duration_s, relative=False, group_mask=ALL_GROUPS
    ):
        """
        Go to an absolute or relative position

        :param x: X (m)
        :param y: Y (m)
        :param z: Z (m)
        :param yaw: Yaw (radians)
        :param duration_s: Time it should take to reach the position (s)
        :param relative: True if x, y, z is relative to the current position
        :param group_mask: Mask for which CFs this should apply to
        """
        packet = self.packer.go_to(group_mask, relative, x, y, z, yaw, duration_s)
        self.link.send_packet_no_response(packet)

    def send_start_trajectory(
        self,
        trajectory_id,
        time_scale=1.0,
        relative=False,
        reversed=False,
        group_mask=ALL_GROUPS,
    ):
        """
        starts executing a specified trajectory

        :param trajectory_id: Id of the trajectory (previously defined by
               define_trajectory)
        :param time_scale: Time factor; 1.0 = original speed;
                                        >1.0: slower;
                                        <1.0: faster
        :param relative: Set to True, if trajectory should be shifted to
               current setpoint
        :param reversed: Set to True, if trajectory should be executed in
               reverse
        :param group_mask: Mask for which CFs this should apply to
        :return:
        """
        packet = self.packer.start_trajectory(
            group_mask, relative, reversed, trajectory_id, time_scale
        )
        self.link.send_packet_no_response(packet)

    def send_define_trajectory(
        self, trajectory_id, offset, n_pieces, type=TRAJECTORY_TYPE_POLY4D
    ):
        """
        Define a trajectory that has previously been uploaded to memory.

        :param trajectory_id: The id of the trajectory
        :param offset: Offset in uploaded memory
        :param n_pieces: Nr of pieces in the trajectory
        :param type: The type of trajectory data; TRAJECTORY_TYPE_POLY4D or TRAJECTORY_TYPE_POLY4D_COMPRESSED
        :return:
        """
        packet = self.packer.define_trajectory(trajectory_id, type, offset, n_pieces)
        self.link.send_packet_no_response(packet)

    def send_takeoff(
        self, absolute_height_m, duration_s, group_mask=ALL_GROUPS, yaw=0.0
    ):
        """
        vertical takeoff from current x-y position to given height

        :param absolute_height_m: Absolute (m)
        :param duration_s: Time it should take until target height is
                           reached (s)
        :param group_mask: Mask for which CFs this should apply to
        :param yaw: Yaw (rad). Use current yaw if set to None.
        """
        target_yaw = yaw
        useCurrentYaw = False
        if yaw is None:
            target_yaw = 0.0
            useCurrentYaw = True
        packet = self.packer.takeoff(
            group_mask, absolute_height_m, target_yaw, useCurrentYaw, duration_s
        )
        self.link.send_packet_no_response(packet)

    def send_land(self, absolute_height_m, duration_s, group_mask=ALL_GROUPS, yaw=0.0):
        """
        vertical land from current x-y position to given height

        :param absolute_height_m: Absolute (m)
        :param duration_s: Time it should take until target height is
                           reached (s)
        :param group_mask: Mask for which CFs this should apply to
        :param yaw: Yaw (rad). Use current yaw if set to None.
        """
        target_yaw = yaw
        useCurrentYaw = False
        if yaw is None:
            target_yaw = 0.0
            useCurrentYaw = True
        packet = self.packer.land(
            group_mask, absolute_height_m, target_yaw, useCurrentYaw, duration_s
        )
        self.link.send_packet_no_response(packet)
