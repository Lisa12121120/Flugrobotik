from rclpy.node import Node

from crtp_driver.crtp_link_ros import CrtpLinkRos
from .crtp_packer_ros import CrtpPackerRos

from crtp.logic.hl_commander_logic import HighLevelCommanderLogic
from crazyflie_interfaces_python.server import HighLevelCommanderServer


class HighLevelCommander(HighLevelCommanderServer, HighLevelCommanderLogic):
    def __init__(self, node: Node, CrtpLink: CrtpLinkRos):
        HighLevelCommanderLogic.__init__(self, CrtpPackerRos, CrtpLink)
        HighLevelCommanderServer.__init__(self, node)

    # Override
    def set_group_mask(self, group_mask: float) -> None:
        self.send_set_group_mask(group_mask)

    # Override
    def go_to(
        self,
        group_mask: int,
        relative: bool,
        linear: bool,
        x: float,
        y: float,
        z: float,
        yaw: float,
        duration_seconds: float,
    ) -> None:
        """
        TODO: Currently there is no implementation for the linear mode
        """
        self.send_go_to(x, y, z, yaw, duration_seconds, relative, group_mask)

    # Override
    def takeoff(
        self,
        group_mask: int,
        height: float,
        yaw: float,
        use_current_yaw: bool,
        duration_seconds: float,
    ) -> None:
        self.send_takeoff(height, duration_seconds, group_mask, yaw)

    # Override
    def land(
        self,
        group_mask: int,
        height: float,
        yaw: float,
        use_current_yaw: bool,
        duration_seconds: float,
    ) -> None:
        self.send_land(height, duration_seconds, group_mask, yaw)

    # Override
    def define_trajectory(self, trajectory_id: int, piece_offset: int) -> None:
        return super().define_trajectory(trajectory_id, piece_offset)

    # Override
    def land_with_velocity(
        self,
        group_mask: int,
        height: float,
        height_is_relative: bool,
        yaw: float,
        use_current_yaw: bool,
        velocity: float,
    ) -> None:
        return super().land_with_velocity(
            group_mask, height, height_is_relative, yaw, use_current_yaw, velocity
        )

    # Override
    def spiral(
        self,
        group_mask: int,
        sidways: bool,
        clockwise: bool,
        phi: float,
        r0: float,
        rf: float,
        dz: float,
        duration_seconds: float,
    ) -> None:
        return super().spiral(
            group_mask, sidways, clockwise, phi, r0, rf, dz, duration_seconds
        )

    # Override
    def start_trajectory(
        self,
        group_mask: int,
        trajectory_id: int,
        timescale: float,
        reversed: bool,
        relative: bool,
    ) -> None:
        return super().start_trajectory(
            group_mask, trajectory_id, timescale, reversed, relative
        )

    # Override
    def stop(self, group_mask: int) -> None:
        return super().stop(group_mask)

    # Override
    def takeoff_with_velocity(
        self,
        group_mask: int,
        height: float,
        height_is_relative: bool,
        yaw: float,
        use_current_yaw: bool,
        velocity: float,
    ) -> None:
        return super().takeoff_with_velocity(
            group_mask, height, height_is_relative, yaw, use_current_yaw, velocity
        )
