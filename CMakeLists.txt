
cmake_minimum_required(VERSION 2.6)

project(rs2-camera-lcm)

find_package(lcm REQUIRED)

find_package(RealSense2 REQUIRED)

include_directories(
    include
    ${REALSENSE2_INCLUDE_DIR}
    )

add_executable(rs2-camera-lcm src/rs2_camera_lcm.cpp)
target_link_libraries(rs2-camera-lcm ${REALSENSE2_LIBRARY} lcm)
