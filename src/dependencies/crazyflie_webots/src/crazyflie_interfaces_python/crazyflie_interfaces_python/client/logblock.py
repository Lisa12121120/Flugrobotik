from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Int16, Empty

from crazyflie_interfaces.msg import GenericLogData

from typing import Callable, List


class LogBlockClient:
    """If a log block was created you can use this to start and stop log blocks."""

    def __init__(
        self,
        node: Node,
        prefix: str,
        name: str,
        callback: Callable[[List[float]], None],
    ):
        self.callback = callback
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile_send = QoSProfile(
            depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        qos_profile_receive = 10
        namespace = "{}/log/{}/".format(prefix, name)

        self.start_log_block_publisher = node.create_publisher(
            Int16,
            namespace + "start",
            qos_profile=qos_profile_send,
            callback_group=callback_group,
        )

        self.stop_log_block_publisher = node.create_publisher(
            Empty,
            namespace + "stop",
            qos_profile=qos_profile_send,
            callback_group=callback_group,
        )

        node.create_subscription(
            GenericLogData,
            namespace + "data",
            callback=self._log_data_callback,
            qos_profile=qos_profile_receive,
            callback_group=callback_group,
        )

    def start_log_block(self, period_ms: int) -> None:
        """Start the log block with specified period.

        Args:
            period_ms (int): The period in ms in which log gets querried
        """
        msg = Int16()
        msg.data = period_ms
        self.start_log_block_publisher.publish(msg)

    def stop_log_block(self) -> None:
        """Stops the log block."""
        msg = Empty()
        self.stop_log_block_publisher.publish(msg)

    def _log_data_callback(self, msg: GenericLogData) -> None:
        self.callback(msg.values)
