from rclpy.node import Node

from crtp_driver.crtp_link_ros import CrtpLinkRos
from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.generic_commander_logic import GenericCommanderLogic
from crazyflie_interfaces_python.server import GenericCommanderServer

from typing import List


class GenericCommander(GenericCommanderServer, GenericCommanderLogic):
    """
    Setpoints are received via ros in the GenericCommanderServer.
    They are then sent out to crtp crazyradio with GenericCommander Logic
    """

    def __init__(self, node: Node, CrtpLink: CrtpLinkRos):
        GenericCommanderLogic.__init__(self, CrtpPackerRos, CrtpLink)
        GenericCommanderServer.__init__(self, node)

    # Override
    def notify_setpoints_stop(self, remain_valid_millisecs: int) -> None:
        self.send_notify_setpoints_stop(remain_valid_millisecs)

    # Override
    def velocity_world_setpoint(
        self, vx: float, vy: float, vz: float, yawrate: float
    ) -> None:
        self.send_velocity_world_setpoint(vx, vy, vz, yawrate)

    # Override
    def hover_setpoint(
        self, vx: float, vy: float, yawrate: float, z_distance: float
    ) -> None:
        self.send_hover_setpoint(vx, vy, yawrate, z_distance)

    # Override
    def full_state_setpoint(
        self,
        position: List[int],
        velocity: List[int],
        acceleration: List[int],
        orienation: List[int],
        angular_rate: List[int],
    ) -> None:
        roll_rate, pitch_rate, yaw_rate = angular_rate
        self.send_full_state_setpoint(
            position,
            velocity,
            acceleration,
            orienation,
            roll_rate,
            pitch_rate,
            yaw_rate,
        )

    # Override
    def position_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        self.send_position_setpoint(x, y, z, yaw)
