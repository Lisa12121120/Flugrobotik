cmake_minimum_required(VERSION 3.5)
project(object_tracker_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTrackerObject.srv"
  "srv/RemoveTrackerObject.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
