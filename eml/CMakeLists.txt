
# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(eml)

include_directories(include ${catkin_INCLUDE_DIRS})
 
catkin_package(
)


install(FILES eml-r36.patch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
