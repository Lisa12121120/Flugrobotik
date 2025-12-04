cmake_minimum_required(VERSION 3.12)
project(crtp_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)


set(${PROJECT_NAME}_SRCS
  src/crtp_cpp/link/crtp_link.cpp
  
  src/crtp_cpp/packer/basic_commander_packer.cpp
  src/crtp_cpp/packer/console_packer.cpp
  src/crtp_cpp/packer/crtp_packer.cpp
  src/crtp_cpp/packer/generic_commander_packer.cpp
  src/crtp_cpp/packer/hl_commander_packer.cpp
  src/crtp_cpp/packer/link_layer_packer.cpp
  src/crtp_cpp/packer/localization_packer.cpp
  src/crtp_cpp/packer/logging_packer.cpp
  src/crtp_cpp/packer/parameters_packer.cpp
  src/crtp_cpp/packer/toc_packer.cpp

  src/crtp_cpp/logic/basic_commander_logic.cpp
  src/crtp_cpp/logic/console_logic.cpp
  src/crtp_cpp/logic/generic_commander_logic.cpp
  src/crtp_cpp/logic/hl_commander_logic.cpp
  src/crtp_cpp/logic/link_layer_logic.cpp
  src/crtp_cpp/logic/localization_logic.cpp
  src/crtp_cpp/logic/logging_logic.cpp
  src/crtp_cpp/logic/logic.cpp
  src/crtp_cpp/logic/parameters_logic.cpp
  src/crtp_cpp/logic/toc_logic.cpp

  src/crtp_cpp/logic/logtocentry.cpp
)  

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})


ament_target_dependencies(${PROJECT_NAME}
  "ament_index_cpp"
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "CRTP_CPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)



# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# specific order: dependents before dependencies
ament_export_dependencies(ament_index_cpp)

ament_package()

install(
  DIRECTORY include/ #${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include/${PROJECT_NAME}
)
ament_generate_version_header(${PROJECT_NAME})

