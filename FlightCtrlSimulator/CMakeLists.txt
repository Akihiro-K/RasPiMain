cmake_minimum_required(VERSION 2.8)
project(main)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

option(FCSIM_DEBUG_MODE "Debug mode" OFF)
if(FCSIM_DEBUG_MODE)
  add_definitions(-DMARKERSIM_DEBUG_MODE)
endif()
MESSAGE("FCSIM_DEBUG_MODE? " ${FCSIM_DEBUG_MODE})

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/utserial ${CMAKE_CURRENT_BINARY_DIR}/utserial_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/serial ${CMAKE_CURRENT_BINARY_DIR}/serial_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/tcp ${CMAKE_CURRENT_BINARY_DIR}/tcp_build)

add_executable(flightctrl_simulator src/flightctrl_simulator.cpp)
target_link_libraries(flightctrl_simulator UTSerial Serial TCPServer)
