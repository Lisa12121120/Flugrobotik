from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.msg import LogBlock
from crazyflie_interfaces_python.client.logblock import LogBlockClient
from typing import Callable, List


class LoggingClient:
    """The logging functionality of the crazyflie.
    https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/crtp_log/

    The logging on the crazyflie is segemented into log blocks.
    Create a log block with variables from: (other implementations (webots or older crazyflie version) might not have
    all logging variables available)
    https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/

    This creates a LogBlockClient, on which you can start and stop the log block.
    """

    def __init__(self, node: Node, prefix: str):
        self.node = node
        self.prefix = prefix
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        self.create_log_block_publisher = node.create_publisher(
            LogBlock,
            prefix + "/create_log_block",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def create_log_block(
        self,
        variables: List[str],
        name: str,
        callback: Callable[[List[float]], None],
    ) -> LogBlockClient:
        """Create a log block with given variables.

        The created topic will have the specified name

        Args:
            variables (List[str]): The logging variables (e.g. ["range.zrange", ])
            name (str): The name of the rostopic
            callback (Callable[[List[float]], None]): A callback function for data beeing received

        Returns:
            LogBlockClient: A LogBlockClient object with which the block can be started/stopped
        """
        msg = LogBlock()
        msg.variables = variables
        msg.name = name
        self.create_log_block_publisher.publish(msg)
        return LogBlockClient(self.node, self.prefix, name, callback)
