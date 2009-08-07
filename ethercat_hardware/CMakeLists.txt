cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ethercat_hardware)

rospack_add_boost_directories()

add_definitions(-Wall)
rospack_add_library(ethercat_hardware src/ethercat_hardware.cpp src/wg0x.cpp src/ek1122.cpp src/wg014.cpp)
rospack_remove_compile_flags(ethercat_hardware -W)

rospack_add_executable(motorconf src/motorconf.cpp src/ethercat_hardware.cpp src/wg0x.cpp src/ek1122.cpp src/wg014.cpp)
target_link_libraries(motorconf loki) 
rospack_remove_compile_flags(motorconf -W)
