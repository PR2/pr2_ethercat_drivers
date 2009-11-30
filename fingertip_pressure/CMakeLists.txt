cmake_minimum_required(VERSION 2.4.6)
set(ROS_BUILD_TYPE RelWithDebInfo)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()

add_definitions(-Wall)

rosbuild_add_rostest(test/pressure_info_tests.xml)
rosbuild_add_rostest(test/test_marker_rectangle.xml)
rosbuild_add_rostest(test/test_marker_sphere.xml)
