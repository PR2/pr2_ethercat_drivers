# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(fingertip_pressure)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
   std_msgs 
   geometry_msgs 
   message_generation
   rostest 
   )


add_message_files(DIRECTORY msg FILES PressureInfoElement.msg PressureInfo.msg)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime)

add_rostest(test/pressure_info_tests.xml)

add_rostest(test/test_marker_rectangle.xml)

add_rostest(test/test_marker_sphere.xml)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(PROGRAMS
   ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES ui/fingertip_panel.xrc
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ui/)

catkin_install_python(PROGRAMS
   test/pressure_info_test.py
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/)
