
cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set (CMAKE_CXX_STANDARD 11)
project(rs2-camera-lcm)

find_package(lcm REQUIRED)

find_package(RealSense2 REQUIRED)

find_package(ZLIB REQUIRED)

include_directories(
    include
    ${REALSENSE2_INCLUDE_DIR}
    ${ZLIB_INCLUDE_DIRS}
    )

add_library(
        rs2_driver_lib
        STATIC
        src/rs2_driver.cpp
    )
target_link_libraries(rs2_driver_lib ${REALSENSE2_LIBRARY} ${ZLIB_LIBRARIES} lcm)
add_executable(rs2-camera-lcm src/rs2_camera_lcm.cpp)
target_link_libraries(rs2-camera-lcm ${REALSENSE2_LIBRARY} ${ZLIB_LIBRARIES} lcm rs2_driver_lib)
