cmake_minimum_required(VERSION 2.8)
project(main)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

option(FC_DEBUG "FC Debug Mode" OFF)
if(FC_DEBUG)
  add_definitions(-DFC_DEBUG_MODE)
endif()

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/utserial ${CMAKE_CURRENT_BINARY_DIR}/utserial_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/serial ${CMAKE_CURRENT_BINARY_DIR}/serial_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/tcp ${CMAKE_CURRENT_BINARY_DIR}/tcp_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/nc ${CMAKE_CURRENT_BINARY_DIR}/nc_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/helper ${CMAKE_CURRENT_BINARY_DIR}/helper_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/Handlers ${CMAKE_CURRENT_BINARY_DIR}/Handlers_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/ublox ${CMAKE_CURRENT_BINARY_DIR}/ublox_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libraries/timer ${CMAKE_CURRENT_BINARY_DIR}/timer_build)


add_executable(main src/main.cpp)
target_link_libraries(main UTSerial Serial TCPClient StateEstimator Navigator Logger Shared Disp Helper Handlers Ublox Timer)
