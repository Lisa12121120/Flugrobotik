from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from crazyflie_interfaces.msg import (
    SetGroupMask,
    Takeoff,
    Land,
    Stop,
    GoTo,
    StartTrajectory,
    UploadTrajectory,
    TakeoffWithVelocity,
    LandWithVelocity,
    Spiral,
    TrajectoryPolynomialPiece,
)

from typing import List


class HighLevelCommanderClient:
    """Send high level commands to the Crazyflie.
    For more information visit the official bitcraze website:
    https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/crazyflie/high_level_commander/

    These high level commands get processed onboard the Crazyflie. Polynomials are calculated in order to fly
    smooth trajectories between the commanded setpoints.
    Before switching from low-level to high-level commands the low-level command notify setpoints stop should be called.
    """

    def __init__(self, node: Node, prefix: str):
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10

        self.set_group_mask_publisher = node.create_publisher(
            SetGroupMask,
            prefix + "/set_group_mask",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.takeoff_publisher = node.create_publisher(
            Takeoff,
            prefix + "/takeoff",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.land_publisher = node.create_publisher(
            Land,
            prefix + "/land",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.stop_publisher = node.create_publisher(
            Stop,
            prefix + "/stop",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.goto_publisher = node.create_publisher(
            GoTo,
            prefix + "/go_to",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.start_trajectory_publisher = node.create_publisher(
            StartTrajectory,
            prefix + "/start_trajectory",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.upload_trajectory_publisher = node.create_publisher(
            UploadTrajectory,
            prefix + "/upload_trajectory",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.takeoff_with_velocity_publisher = node.create_publisher(
            TakeoffWithVelocity,
            prefix + "/takeoff_with_velocity",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.land_with_velocity_publisher = node.create_publisher(
            LandWithVelocity,
            prefix + "/land_with_velocity",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.spiral_publisher = node.create_publisher(
            Spiral,
            prefix + "/spiral",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def set_group_mask(self, group_mask: int):
        """Sets the group mask of the crazyflie

        Deprecated will be removed December 2024
        This can be used to split a swarm of Crazyflies into groups and then send high level commands via broadcasting messages.

        Args:
            group_mask (int): the group ID this CF belongs to
        """
        msg = SetGroupMask()
        msg.group_mask = group_mask
        self.set_group_mask_publisher.publish(msg)

    def takeoff(
        self,
        target_height: float,
        duration_seconds: float,
        yaw: float = None,
        use_current_yaw: bool = False,
        group_mask: int = 0,
    ) -> None:
        """Vertical takeoff from current x-y position to given height (high-level)

        The Crazyflie will hover indefinetely after target_height is reached.

        Args:
            target_height (float): height to takeoff to (absolute) in meters
            duration_seconds (float): time it should take until target_height is reached in seconds
            yaw (float): Target orientation in radians
            use_current_yaw (bool): If true use ignore yaw parameter. Defaults to False.
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        msg = Takeoff()
        msg.group_mask = group_mask
        msg.height = target_height
        msg.use_current_yaw = use_current_yaw
        if not yaw is None:
            msg.yaw = yaw
        msg.duration = self.__seconds_to_duration(duration_seconds)
        self.takeoff_publisher.publish(msg)

    def land(
        self,
        target_height: float,
        duration_seconds: float,
        yaw: float = None,
        group_mask: int = 0,
    ) -> None:
        """Vertical landing from current x-y position to given height (high-level)

        The Crazyflie will hover indefinetely after target_height is reached.
        This should usually be followed by a stop command, but is not strictly required.

        Args:
            target_height (float): _description_
            duration_seconds (float): _description_
            yaw (float, optional): _description_. Defaults to None.
            group_mask (int, optional): _description_. Defaults to 0.
        """
        msg = Land()
        msg.group_mask = group_mask
        msg.height = target_height
        msg.use_current_yaw = yaw is None
        if not msg.use_current_yaw:
            msg.yaw = yaw
        msg.duration = self.__seconds_to_duration(duration_seconds)
        self.land_publisher.publish(msg)

    def stop(self, group_mask: int = 0) -> None:
        """Turns off the motors (high-level)

        Args:
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        msg = Stop()
        msg.group_mask = group_mask
        self.stop_publisher.publish(msg)

    def go_to(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        duration_seconds: float,
        relative: bool = False,
        linear: bool = False,
        group_mask: int = 0,
    ) -> None:
        """Move to x, y, z, yaw in duration_seconds amount of time (high-level)

        The Crazyflie will hover indefinetely afterwards.
        Calling goTo rapidly (> 1Hz) can cause instability. Consider using the cmd_position() setpoint command from
        generic_commander instead.

        Args:
            x (float): x-position of goal in meters
            y (float): y-position of goal in meters
            z (float): z-position of goal in meters
            yaw (float): target yaw in radians
            duration_seconds (float): Time in seconds it should take the CF to move to goal
            relative (bool, optional): If true the goal and yaw are interpreted as relative to current position. Defaults to False.
            linear (bool, optional): If true a linear interpolation is used for trajectory instead of a smooth polynomial . Defaults to False.
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        msg = GoTo()
        msg.group_mask = group_mask
        msg.goal = Point(x=x, y=y, z=z)
        msg.yaw = yaw
        msg.relative = relative
        msg.linear = linear
        msg.duration = self.__seconds_to_duration(duration_seconds)
        self.goto_publisher.publish(msg)

    def start_trajectory(
        self,
        trajectory_id: int,
        timescale: float = 1.0,
        reversed: bool = False,
        relative: bool = True,
        group_mask: int = 0,
    ) -> None:
        """Begin executing an uploaded trajectory (high-level)

        Args:
            trajectory_id (int): ID of trajectory as uploaded
            timescale (float, optional): Scales the duration of trajectory by this factor (if 2.0, trajectory twice as long). Defaults to 1.0.
            reversed (bool, optional): Execute the trajectory in reverse order. Defaults to False.
            relative (bool, optional): Of true, the position of the trajectory is shifted such that it begins at the current position setpoint. Defaults to True.
            group_mask (int, optional): mask for which Crazyflies this should apply to. Defaults to 0.
        """
        msg = StartTrajectory()
        msg.group_mask = group_mask
        msg.trajectory_id = trajectory_id
        msg.timescale = timescale
        msg.reversed = reversed
        msg.relative = relative
        self.start_trajectory_publisher.publish(msg)

    def upload_trajectory(
        self, trajectory_id: int, piece_offset: int, pieces: List
    ) -> None:
        """Uploads a piecewise polynomial trajectory for later execution.

        This feature is currently not fully supported.

        TODO: How do poly pieces work, should we also use uav_trajectory.py
        See https://crazyswarm.readthedocs.io/en/latest/api.html or ask whoenig for further information.

        Args:
            trajectory_id (int): The trajectory id to reference in start_trajectory
            piece_offset (int): TODO
            pieces (List): TODO
        """
        msg = UploadTrajectory()
        msg.trajectory_id = trajectory_id
        msg.piece_offset = piece_offset
        for _piece in pieces:
            piece = TrajectoryPolynomialPiece()
            piece.duration = self.__seconds_to_duration(_piece.duration_seconds)
            piece.poly_x = _piece.poly_x
            piece.poly_y = _piece.poly_y
            piece.poly_z = _piece.poly_z
            piece.poly_yaw = _piece.poly_yaw
            msg.pieces.append(piece)
        self.upload_trajectory_publisher.publish(msg)

    def takeoff_with_velocity(
        self,
        height: float,
        yaw: float,
        velocity: float,
        height_is_relative: bool = False,
        use_current_yaw: bool = False,
        group_mask: int = 0,
    ) -> None:
        """Vertical takeoff with given velocity (high-level)
        Only supported in newer firmware versions.

        Args:
            height (float): Height to takeoff to in m (absolute unless height_is_relative is set)
            yaw (float): Target orientation in radians
            velocity (float): Average velocity during takeoff m/s
            height_is_relative (bool, optional): If true height is relative to current zHeight. Defaults to False.
            use_current_yaw (bool, optional): If true ignore yaw parameter and use current orientation. Defaults to False.
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        msg = TakeoffWithVelocity()
        msg.group_mask = group_mask
        msg.height = height
        msg.height_is_relative = height_is_relative
        msg.yaw = yaw
        msg.velocity = velocity
        msg.use_current_yaw = use_current_yaw
        self.takeoff_with_velocity_publisher.publish(msg)

    def land_with_velocity(
        self,
        height: float,
        yaw: float,
        velocity: float,
        height_is_relative: bool = False,
        use_current_yaw: bool = False,
        group_mask: int = 0,
    ) -> None:
        """Vertical landing with given velocity (high-level)
        Only supported in newer firmware versions.

        Args:
            height (float): Height to land to in m (absolute unless height_is_relative is set)
            yaw (float): Target orientation in radians
            velocity (float): Average velocity during takeoff m/s
            height_is_relative (bool, optional): If true height is relative to current zHeight. Defaults to False.
            use_current_yaw (bool, optional): If true ignores yaw parameter and use current orientation. Defaults to False.
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        msg = LandWithVelocity()
        msg.group_mask = group_mask
        msg.height = height
        msg.height_is_relative = height_is_relative
        msg.yaw = yaw
        msg.use_current_yaw = use_current_yaw
        msg.velocity = velocity
        self.land_with_velocity_publisher.publish(msg)

    def spiral(
        self,
        sideways: bool,
        clockwise: bool,
        phi: float,
        r0: float,
        rf: float,
        dz: float,
        duration_seconds: float,
        group_mask: int = 0,
    ) -> None:
        """Fly along a spiral path (high-level)

        Args:
            sideways (bool): Set to true if Crazyflie shoud spiral sideways instead of forward
            clockwise (bool): Set to true if Crazyflie shoudl spiral clockwise instead of counter-clockwise
            phi (float): Spiral angle in radians, limited to +- 2 PI
            r0 (float): Inital radius in m, must be positive
            rf (float): Final radius in m, must be positive
            dz (float): Altitude gain in , if positive climb else descent
            duration_seconds (float): The time it should take to reach the end of the spiral in seconds
            group_mask (int, optional): mask for which crazyflies this should apply to. Defaults to 0.
        """
        msg = Spiral()
        msg.group_mask = group_mask
        msg.sideways = sideways
        msg.clockwise = clockwise
        msg.phi = phi
        msg.r0 = r0
        msg.rf = rf
        msg.dz = dz
        msg.duration = self.__seconds_to_duration(duration_seconds)
        self.spiral_publisher.publish(msg)

    def __seconds_to_duration(self, seconds: float) -> Duration:
        i_seconds = int(seconds)
        fractional_seconds = seconds - i_seconds
        nanoseconds = int(fractional_seconds * 1_000_000_000)
        duration = Duration()
        duration.sec = i_seconds
        duration.nanosec = nanoseconds        
        return duration
