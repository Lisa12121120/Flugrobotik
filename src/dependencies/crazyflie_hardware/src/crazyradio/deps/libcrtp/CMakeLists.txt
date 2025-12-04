cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(libcrtp)

include_directories(include)
add_library(libcrtp 
        CrtpPacketQueue.cpp
        CrtpLink.cpp
        CrtpLinkContainer.cpp
        CrtpLogger.cpp)

