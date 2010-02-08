cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_add_boost_directories()
rosbuild_genmsg()

add_definitions(-Wall)
rosbuild_add_library(ethercat_hardware src/ethercat_hardware.cpp src/ethercat_com.cpp src/ethercat_device.cpp src/wg0x.cpp src/ek1122.cpp src/wg014.cpp src/motor_model.cpp)
rosbuild_remove_compile_flags(ethercat_hardware -W)

rosbuild_add_executable(motorconf src/motorconf.cpp src/ethercat_hardware.cpp src/ethercat_com.cpp src/ethercat_device.cpp src/wg0x.cpp src/ek1122.cpp src/wg014.cpp src/motor_model.cpp)
target_link_libraries(motorconf rt)
rosbuild_link_boost (motorconf system)
rosbuild_remove_compile_flags(motorconf -W)

