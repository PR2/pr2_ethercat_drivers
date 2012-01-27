cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

find_library(LOG4CXX_LIBRARY log4cxx)
rosbuild_add_boost_directories()

rosbuild_add_boost_directories()
rosbuild_genmsg()
rosbuild_gensrv()

add_definitions(-Wall)
rosbuild_add_library(
  ethercat_hardware src/ethercat_hardware.cpp src/ethercat_com.cpp src/ethercat_device.cpp 
  src/wg0x.cpp src/wg05.cpp src/wg06.cpp src/wg021.cpp src/ek1122.cpp src/wg014.cpp 
  src/motor_model.cpp src/ethernet_interface_info.cpp src/motor_heating_model.cpp 
  src/wg_soft_processor.cpp src/wg_util.cpp src/wg_mailbox.cpp src/wg_eeprom.cpp
  )

rosbuild_add_executable(motorconf 
  src/motorconf.cpp src/ethercat_hardware.cpp src/ethercat_com.cpp src/ethercat_device.cpp 
  src/wg0x.cpp src/wg05.cpp src/wg06.cpp src/wg021.cpp src/ek1122.cpp src/wg014.cpp 
  src/motor_model.cpp src/ethernet_interface_info.cpp src/motor_heating_model.cpp
  src/wg_soft_processor.cpp src/wg_util.cpp src/wg_mailbox.cpp src/wg_eeprom.cpp
  )

target_link_libraries(motorconf rt tinyxml ${LOG4CXX_LIBRARY})
rosbuild_link_boost(motorconf regex system filesystem thread)

rosbuild_add_gtest(test/wg0x_test test/wg0x_test.cpp )
target_link_libraries(test/wg0x_test ethercat_hardware tinyxml)

rosbuild_add_gtest(test/motor_heating_model_test test/motor_heating_model_test.cpp )
target_link_libraries(test/motor_heating_model_test ethercat_hardware tinyxml)
