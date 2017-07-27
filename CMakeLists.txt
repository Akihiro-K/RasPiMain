cmake_minimum_required(VERSION 2.8)
project(main)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_subdirectory(src/myserial)
add_subdirectory(src/mytcp)
add_subdirectory(src/stateestimator)
add_subdirectory(src/navigator)
add_subdirectory(src/logger)
add_subdirectory(src/shared)
add_subdirectory(src/disp)

add_executable(main src/main.cpp)
target_link_libraries(main MySerial MyTCPServer MyTCPClient StateEstimator Navigator Logger Shared Disp)
