cmake_minimum_required(VERSION 3.8)
project(crtp_cpp_test)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crtp_cpp REQUIRED)

add_executable(test_node 
    src/test_node.cpp)

ament_target_dependencies(test_node
  crtp_cpp
  rclcpp)


install(TARGETS 
  test_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
