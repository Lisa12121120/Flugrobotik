#!/usr/bin/env python3
import os
import asyncio
from asyncio.subprocess import Process

from threading import Thread
from signal import SIGINT

from ament_index_python.packages import get_package_share_directory
from ros2run.api import get_executable_path

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException

from crazyflie_hardware_gateway_interfaces.srv import AddCrazyflie, RemoveCrazyflie
from geometry_msgs.msg import Point
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State as LifecycleState, TransitionEvent

from typing import Dict, Tuple, List, Optional
from dataclasses import dataclass
from threading import Thread
import time


class GatewayError(Exception):
    pass


@dataclass
class CrazyflieInstance:
    get_state: Client
    change_state: Client
    process: Optional[Process] = None


class Gateway(Node):
    def __init__(self, event_loop):
        super().__init__(
            "crazyflie_hardware_gateway",
            automatically_declare_parameters_from_overrides=True,
        )
        self._event_loop = event_loop
        self._logger = self.get_logger()
        self._logger.info("Started Hardware-Gateway")

        self.crazyflies: Dict[Tuple[int, int], CrazyflieInstance] = {}
        self.crazyflie_callback_group = MutuallyExclusiveCallbackGroup()

        callback_group = MutuallyExclusiveCallbackGroup()
        self.add_service = self.create_service(
            srv_type=AddCrazyflie,
            srv_name="~/add_crazyflie",
            callback=self._add_crazyflie_callback,
            callback_group=callback_group,
        )
        self.remove_service = self.create_service(
            srv_type=RemoveCrazyflie,
            srv_name="~/remove_crazyflie",
            callback=self._remove_crazyflie_callback,
            callback_group=callback_group,
        )

    def remove_crazyflie(self, channel: int, id: int) -> Tuple[bool, str]:
        """Remove a crazyflie.

        Removes the crazyflie. This Stops the Crazyflies process with a SIGINT.
        The crazyflie then automatically closes its connection to the radio.

        Args:
            channel (int): The channel of the crazyflie
            id (int): The id of the crazyflie

        Returns:
            bool: Returns False if crazyflie was not in the list of started crazyflies.
        """
        self.get_logger().info("Removing Crazyflie with ID: {}".format(id))
        if (channel, id) in self.crazyflies.keys():
            self.destroy_client(self.crazyflies[(channel, id)].change_state)
            self.destroy_client(self.crazyflies[(channel, id)].get_state)

            process = self.crazyflies[(channel, id)].process
            os.killpg(os.getpgid(process.pid), SIGINT)

            del self.crazyflies[(channel, id)]
            return (True, "Success")
        msg = (
            "Couldn't remove crazyflie with Channel: {}, ID: {}; was not active".format(
                channel, id
            )
        )
        self._logger.info(msg)
        return (False, msg)

    def remove_all_crazyflies(self):
        """Removes all crazyflies which were started with the gateway."""
        for cf_key in list(self.crazyflies.keys()):
            _ = self.remove_crazyflie(*cf_key)

    def add_crazyflie(
        self,
        channel: int,
        id: int,
        initial_position: Point,
        type: str,
    ) -> bool:
        """Adds a crazyflie with given configuration.

        Creates and holds a process to a crazyflie.

        Args:
            channel (int): The channel the crayflie is on
            id (int): The id of the crazyflie
            initial_position (Point): The expected initial position
            type (str): The type as referencened in a .yaml (stores marker and dynamics Conf idx...)
        Returns:
            bool: True if the crazyflie could be created. False if we already have a crazyflie with same id/channel
        """
        self._logger.info("Got called to add Crazyflie with ID: {}".format(id))
        if (channel, id) in self.crazyflies.keys():
            self._logger.debug("Crazyflie already in gateway. ID: {}".format(id))
            cf_state: Optional[LifecycleState] = self._get_state(channel, id)
            if cf_state is not None:
                if cf_state.id == LifecycleState.PRIMARY_STATE_UNCONFIGURED:
                    success = self._transition_crazyflie(
                        channel,
                        id,
                        LifecycleState.TRANSITION_STATE_CONFIGURING,
                        "configure",
                    )
                    return success
                if cf_state.id == LifecycleState.PRIMARY_STATE_ACTIVE:
                    # Called to add a crazyflie which is already properly initialized.
                    return True
            raise GatewayError("Cannot add Crazyflie, is already in Gateway!")
        else:
            wait_future = asyncio.run_coroutine_threadsafe(
                self._create_cf(
                    channel,
                    id,
                    initial_position,
                    type,
                ),
                loop=self._event_loop,
            )
            wait_future.add_done_callback(
                lambda fut: self._on_crazyflie_exit(fut, channel, id)
            )

            key = (channel, id)
            if self._wait_for_change_state_service(key, timeout=3.0):
                success = self._transition_crazyflie(
                    channel,
                    id,
                    LifecycleState.TRANSITION_STATE_CONFIGURING,
                    "configure",
                )
                return success
            else:
                raise GatewayError(
                    f"Crazyflie {key} did not provide change_state service."
                )

    def _on_crazyflie_exit(self, fut: asyncio.Future, channel: int, id: int):
        """Callback invoked when a crazyflie subprocess exits."""
        self._logger.info(f"Crazyflie (channel={channel}, id={id}) exited.")

        # Clean up the crazyflie entry from the dictionary
        if (channel, id) in self.crazyflies.keys():
            self.destroy_client(self.crazyflies[(channel, id)].change_state)
            self.destroy_client(self.crazyflies[(channel, id)].get_state)
            del self.crazyflies[(channel, id)]

    async def _create_cf(
        self,
        channel: int,
        id: int,
        initial_position: List[float],
        type: str,
    ):
        change_state_client = self.create_client(
            srv_type=ChangeState,
            srv_name=f"cf{id}/change_state",
            callback_group=self.crazyflie_callback_group,
        )
        get_state_client = self.create_client(
            srv_type=GetState,
            srv_name=f"cf{id}/get_state",
            callback_group=self.crazyflie_callback_group,
        )

        cmd = self._create_start_command(
            channel,
            id,
            initial_position,
            type,
        )

        process = await asyncio.create_subprocess_exec(*cmd, preexec_fn=os.setsid)

        self.crazyflies[(channel, id)] = CrazyflieInstance(
            get_state_client, change_state_client, process
        )

        await self.crazyflies[(channel, id)].process.wait()

    def _create_start_command(
        self,
        channel: int,
        id: int,
        initial_position: List[float],
        type: str,
    ) -> List[str]:
        crazyflie_path = get_executable_path(
            package_name=self.crazyflie_implementation_package,
            executable_name="crazyflie",
        )

        send_external_position = self.__get_send_external_position(type)
        send_external_pose = self.__get_send_external_pose(type)
        max_initial_deviation = self.__get_max_initial_deviation(type)
        marker_configuration_index = self.__get_marker_configuration_index(type)
        dynamics_configuration_index = self.__get_dynamics_configuration_index(type)

        cmd = [crazyflie_path, "--ros-args"]

        def add_parameter(name: str, value: str):
            cmd.append("-p")
            cmd.append(f"{name}:={value}")

        add_parameter("id", str(id))
        add_parameter("channel", str(channel))
        add_parameter("datarate", str(2))
        add_parameter("initial_position", "[{},{},{}]".format(*initial_position))
        add_parameter("send_external_position", str(send_external_position))
        add_parameter("send_external_pose", str(send_external_pose))
        add_parameter("max_initial_deviation", str(max_initial_deviation))
        add_parameter("marker_configuration_index", str(marker_configuration_index))
        add_parameter("dynamics_configuration_index", str(dynamics_configuration_index))

        cmd += [
            "--params-file",
            self.crazyflie_configuration_yaml,
            "-r",
            "__node:=cf{}".format(id),
        ]

        return cmd

    def _add_crazyflie_callback(
        self, req: AddCrazyflie.Request, resp: AddCrazyflie.Response
    ) -> AddCrazyflie.Response:
        try:
            resp.success = self.add_crazyflie(
                req.channel,
                req.id,
                [
                    req.initial_position.x,
                    req.initial_position.y,
                    req.initial_position.z,
                ],
                req.type,
            )
            if not resp.success:
                resp.msg = "See Crazyflie log for more detail!"
                self.remove_crazyflie(req.channel, req.id)
        except (TimeoutError, GatewayError) as ex:
            self.remove_crazyflie(req.channel, req.id)
            resp.success = False
            resp.msg = str(ex)

        return resp

    def _remove_crazyflie_callback(
        self, req: RemoveCrazyflie.Request, resp: RemoveCrazyflie.Response
    ) -> RemoveCrazyflie.Response:
        resp.success, resp.msg = self.remove_crazyflie(req.channel, req.id)
        return resp

    def __get_send_external_position(self, type: str) -> bool:
        name = "sendExternalPosition"
        return self.__get_typed_parameter_value(type, name).bool_value

    def __get_send_external_pose(self, type: str) -> bool:
        name = "sendExternalPose"
        return self.__get_typed_parameter_value(type, name).bool_value

    def __get_max_initial_deviation(self, type: str) -> float:
        name = "maxInitialDeviation"
        return self.__get_typed_parameter_value(type, name).double_value

    def __get_marker_configuration_index(self, type: str) -> int:
        name = "markerConfigurationIndex"
        return self.__get_typed_parameter_value(type, name).integer_value

    def __get_dynamics_configuration_index(self, type: str) -> int:
        name = "dynamicsConfigurationIndex"
        return self.__get_typed_parameter_value(type, name).integer_value

    def __get_typed_parameter_value(self, type: str, name: str):
        return self.get_parameter(f"crazyflieTypes.{type}.{name}").get_parameter_value()

    @property
    def crazyflie_configuration_yaml(self) -> str:
        return (
            self.get_parameter("crazyflie_configuration_yaml")
            .get_parameter_value()
            .string_value
        )

    @property
    def crazyflie_implementation_package(self) -> str:
        impl = self.get_parameter("implementation").get_parameter_value().string_value
        if impl == "cpp":
            return "crazyflie_hardware_cpp"
        else:
            return "crazyflie_hardware"

    def _wait_for_change_state_service(
        self, key: Tuple[int, int], timeout: float
    ) -> bool:
        while timeout > 0.0:
            if (
                key in self.crazyflies.keys()
                and self.crazyflies[key].change_state.service_is_ready()
            ):
                return True

            time.sleep(0.02)
            timeout -= 0.02
        return False

    def _wait_for_get_state_service(self, key: Tuple[int, int], timeout: float) -> bool:
        while timeout > 0.0:
            if (
                key in self.crazyflies.keys()
                and self.crazyflies[key].get_state.service_is_ready()
            ):
                return True

            time.sleep(0.02)
            timeout -= 0.02
        return False

    def _transition_crazyflie(
        self, channel: int, cf_id: int, state: LifecycleState, label: str
    ) -> bool:
        request = ChangeState.Request()
        request.transition.id = state
        request.transition.label = label
        if (channel, cf_id) in self.crazyflies.keys():
            self._wait_for_change_state_service((channel, cf_id), 0.2)
            fut = self.crazyflies[(channel, cf_id)].change_state.call_async(request)
            timeout = 0.0
            while not fut.done() and timeout < 10.0:
                rclpy.spin_until_future_complete(node=self, future=fut, timeout_sec=0.1)
                if not self._wait_for_change_state_service((channel, cf_id), 0.1):
                    raise TimeoutError("Crazyflie died during transition.")
                timeout += 0.1

            if fut.done():
                response: Optional[ChangeState.Response] = fut.result()
                response: ChangeState.Response
                return response.success

            raise TimeoutError("Service call for transition timed out.")
        else:
            raise GatewayError(
                f"Crazyflie with ch: {channel}, id: {cf_id} not available."
            )

    def _get_state(self, channel: int, cf_id: int) -> Optional[LifecycleState]:
        request = GetState.Request()
        if (channel, cf_id) in self.crazyflies.keys():
            self._wait_for_get_state_service((channel, cf_id), 0.2)
            fut: Future = self.crazyflies[(channel, cf_id)].get_state.call_async(
                request
            )
            rclpy.spin_until_future_complete(node=self, future=fut, timeout_sec=1.0)
            response: Optional[GetState.Response] = fut.result()
            if fut.done():
                response: GetState.Response
                return response.current_state
            else:
                raise TimeoutError("Service call for getting the state timed out.")


async def run_node(cf_eventloop):
    rclpy.init()
    gateway = Gateway(cf_eventloop)
    try:
        while rclpy.ok():
            await asyncio.sleep(0.01)
            rclpy.spin_once(gateway, timeout_sec=0)
        rclpy.shutdown()
    except (asyncio.CancelledError, ExternalShutdownException):
        gateway.remove_all_crazyflies()


def run_crazyflie_loop(event_loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(event_loop)

    async def keep_alive():
        try:
            while True:
                await asyncio.sleep(1)  # Keep the loop alive
        except asyncio.CancelledError:
            pass

    alive_task = asyncio.ensure_future(keep_alive())
    # If there is no future in the loop it cannot be stopped...

    event_loop.run_forever()
    alive_task.cancel()
    for fut in asyncio.all_tasks(event_loop):
        event_loop.run_until_complete(fut)
    event_loop.close()


def run_gateway_loop(
    event_loop: asyncio.AbstractEventLoop,
    crazyflie_event_loop: asyncio.AbstractEventLoop,
):
    asyncio.set_event_loop(event_loop)
    node = asyncio.ensure_future(run_node(crazyflie_event_loop), loop=event_loop)
    event_loop.run_forever()
    node.cancel()
    event_loop.run_until_complete(node)
    event_loop.close()


def main():
    gateway_event_loop = asyncio.new_event_loop()
    crazyflie_event_loop = asyncio.new_event_loop()

    crazyflie_thread = Thread(target=run_crazyflie_loop, args=(crazyflie_event_loop,))
    gateway_thread = Thread(
        target=run_gateway_loop, args=(gateway_event_loop, crazyflie_event_loop)
    )
    crazyflie_thread.start()
    gateway_thread.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        crazyflie_event_loop.stop()
        gateway_event_loop.stop()

    print("This")
    crazyflie_thread.join()
    print("That")
    gateway_thread.join()
    print("What")
    exit()


if __name__ == "__main__":
    main()
