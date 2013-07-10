# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(fingertip_pressure)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS pr2_msgs geometry_msgs visualization_msgs rostest message_generation)


add_message_files(DIRECTORY msg FILES PressureInfoElement.msg PressureInfo.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS pr2_msgs geometry_msgs visualization_msgs message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_rostest(test/pressure_info_tests.xml)

add_rostest(test/test_marker_rectangle.xml)

add_rostest(test/test_marker_sphere.xml)

