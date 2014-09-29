cmake_minimum_required(VERSION 2.8.3)
project(ethercat_hardware)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS pr2_msgs pr2_hardware_interface roscpp realtime_tools diagnostic_msgs diagnostic_updater pluginlib message_generation)

find_library(LOG4CXX_LIBRARY log4cxx)

find_package(EML REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EML_INCLUDE_DIRS})

add_definitions(-Wall)

add_message_files( DIRECTORY msg FILES
ActuatorInfo.msg
BoardInfo.msg
MotorTemperature.msg
MotorTrace.msg
MotorTraceSample.msg
RawFTData.msg
RawFTDataSample.msg
)

add_service_files(DIRECTORY srv FILES
SoftProcessorFirmwareRead.srv
SoftProcessorFirmwareWrite.srv
SoftProcessorReset.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    DEPENDS EML
    CATKIN_DEPENDS pr2_msgs pr2_hardware_interface roscpp realtime_tools diagnostic_msgs diagnostic_updater pluginlib message_runtime
    INCLUDE_DIRS include
    LIBRARIES ethercat_hardware
)

add_definitions(-O3)

add_library(
  ethercat_hardware src/ethercat_hardware.cpp src/ethercat_com.cpp
  src/ethercat_device.cpp src/wg0x.cpp src/wg05.cpp src/wg06.cpp src/wg021.cpp
  src/ek1122.cpp src/wg014.cpp src/motor_model.cpp
  src/ethernet_interface_info.cpp src/motor_heating_model.cpp 
  src/wg_soft_processor.cpp src/wg_util.cpp src/wg_mailbox.cpp src/wg_eeprom.cpp
  )
add_dependencies(ethercat_hardware ${ethercat_hardware_EXPORTED_TARGETS})
target_link_libraries(ethercat_hardware ${catkin_LIBRARIES})
pr2_enable_rpath(ethercat_hardware)

add_executable(motorconf 
  src/motorconf.cpp src/ethercat_hardware.cpp src/ethercat_com.cpp
  src/ethercat_device.cpp src/wg0x.cpp src/wg05.cpp src/wg06.cpp src/wg021.cpp
  src/ek1122.cpp src/wg014.cpp src/motor_model.cpp
  src/ethernet_interface_info.cpp src/motor_heating_model.cpp
  src/wg_soft_processor.cpp src/wg_util.cpp src/wg_mailbox.cpp src/wg_eeprom.cpp
  )
add_dependencies(motorconf ${ethercat_hardware_EXPORTED_TARGETS})

message(STATUS ${LD_LIBRARY_PATH})
target_link_libraries(motorconf rt tinyxml ${LOG4CXX_LIBRARY} ${EML_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS regex system thread filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(motorconf ${Boost_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(wg0x_test test/wg0x_test.cpp )
target_link_libraries(wg0x_test ethercat_hardware tinyxml ${EML_LIBRARIES})
add_dependencies(wg0x_test ${ethercat_hardware_EXPORTED_TARGETS})

catkin_add_gtest(motor_heating_model_test test/motor_heating_model_test.cpp )
target_link_libraries(motor_heating_model_test ethercat_hardware tinyxml
   ${EML_LIBRARIES})
add_dependencies(motor_heating_model_test ${ethercat_hardware_EXPORTED_TARGETS})

install(TARGETS ethercat_hardware
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS motorconf
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES ethercat_device_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
