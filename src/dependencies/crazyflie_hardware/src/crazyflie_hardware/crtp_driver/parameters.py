import rclpy
from rclpy.node import Node
from .crtp_link_ros import CrtpLinkRos
import os

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Empty, Int16


from .crtp_packer_ros import CrtpPackerRos
from crtp.logic.parameters_logic import ParametersLogic
from crazyflie_interfaces_python.server.parameters import ParametersServer

from typing import Dict
from numbers import Number


class Parameters(ParametersServer, ParametersLogic):
    def __init__(self, node: Node, crtp_link: CrtpLinkRos):
        p = os.environ.get("HOME")
        path = os.path.join(p, ".crazyflie", "param")
        ParametersLogic.__init__(self, CrtpPackerRos, crtp_link, path)
        ParametersServer.__init__(self, node)

        self.initialize_toc()

        # node.add_on_set_parameters_callback(self.set_parameter_callback)

    # Override
    def download_toc(self) -> None:
        self.send_download_toc_items()

    # Override
    def get_toc_info(self) -> None:
        nbr_of_items, crc = self.send_get_toc_info()
        self.node.get_logger().info(str("NBR of Items: " + str(nbr_of_items)))

    # Overrite
    def set_parameter(self, group: str, name: str, value: Number):
        self.send_set_parameter(group, name, value)

    def initialize_toc(self):
        super().initialize_toc()
        parameters: Dict[str, rclpy.Parameter.Type] = {}
        for group in self.toc.toc:
            for name in self.toc.toc[group]:
                toc_element = self.toc.get_element(group, name)
                if toc_element.pytype == "<f":
                    par_type = rclpy.Parameter.Type.DOUBLE
                else:
                    par_type = rclpy.Parameter.Type.INTEGER
                parameters[str(group) + "." + str(name)] = par_type
        self.initalize_parameters(parameters)
