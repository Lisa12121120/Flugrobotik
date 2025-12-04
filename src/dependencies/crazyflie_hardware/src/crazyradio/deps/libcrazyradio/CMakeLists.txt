cmake_minimum_required(VERSION 3.5)
project(libcrazyradio)
find_library(USB_LIB usb-1.0)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../libcrtp ${CMAKE_CURRENT_BINARY_DIR}/../libcrtp)

option(LEGACY_RADIO "Use legacy radio-usb protocol" OFF)

if(LEGACY_RADIO)
  add_definitions(-DLEGACY_RADIO)
  message(STATUS "Using legacy radio-usb protocol")
else()
  message(STATUS "Using new radio-usb protocol")
endif()

include_directories(include)
add_library(libcrazyradio 
        Crazyradio.cpp
        USBDevice.cpp)

target_link_libraries(libcrazyradio
    libcrtp
    ${USB_LIB})

target_include_directories(libcrazyradio PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../libcrtp/include)
