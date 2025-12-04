cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)

project(crazyradio)

add_subdirectory(deps/libcrazyradio)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(crtp_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  deps/libcrazyradio/include
)


add_executable(crazyradio_node 
  src/crazyradio_node.cpp)

#dependencies for node from ros packages
ament_target_dependencies(crazyradio_node
  rclcpp
  crtp_interfaces)

#dependencies for node from local libs
target_link_libraries(crazyradio_node
  libcrazyradio)

install(TARGETS   
  crazyradio_node 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
