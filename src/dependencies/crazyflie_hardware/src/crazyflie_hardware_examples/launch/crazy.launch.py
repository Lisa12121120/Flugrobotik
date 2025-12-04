from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    crazyradio_cpp = Node(
        package="crazyradio_cpp",
        executable="crazyradio_node"
    )

    broadcaster = Node(
        package="crtp_driver",
        executable="crtp_broadcaster"
    )

    radiolistener = Node(
        package="crtp_driver",
        executable="radiolistener"
    )

    return LaunchDescription([
        crazyradio_cpp,
        radiolistener,
        broadcaster
    ])

