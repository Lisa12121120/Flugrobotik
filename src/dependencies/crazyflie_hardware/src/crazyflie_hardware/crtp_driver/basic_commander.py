from rclpy.node import Node

from crtp_driver.crtp_link_ros import CrtpLinkRos
from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.basic_commander_logic import BasicCommanderLogic
from crazyflie_interfaces_python.server import RPYTCommanderServer


class BasicCommander(RPYTCommanderServer, BasicCommanderLogic):
    def __init__(self, node: Node, CrtpLink: CrtpLinkRos):
        BasicCommanderLogic.__init__(self, CrtpPackerRos, CrtpLink)
        RPYTCommanderServer.__init__(self, node)

    def setpoint(self, roll: float, pitch: float, yawrate: float, thrust: int) -> None:
        self.send_setpoint(roll, pitch, yawrate, thrust)
