/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <iomanip>

#include <algorithm>

#include <math.h>
#include <stddef.h>

#include <ethercat_hardware/wg06.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <boost/static_assert.hpp>

#include "ethercat_hardware/wg_util.h"

PLUGINLIB_EXPORT_CLASS(WG06, EthercatDevice);


WG06::WG06() :
  has_accel_and_ft_(false),
  pressure_checksum_error_(false),
  pressure_checksum_error_count_(0),
  accelerometer_samples_(0), 
  accelerometer_missed_samples_(0),
  first_publish_(true),
  last_pressure_time_(0),
  pressure_publisher_(NULL),
  accel_publisher_(NULL),
  ft_overload_limit_(31100),
  ft_overload_flags_(0),
  ft_disconnected_(false),
  ft_vhalf_error_(false),
  ft_sampling_rate_error_(false),
  ft_sample_count_(0),
  ft_missed_samples_(0),
  diag_last_ft_sample_count_(0),
  raw_ft_publisher_(NULL),
  ft_publisher_(NULL),
  enable_pressure_sensor_(true),
  enable_ft_sensor_(false),
  enable_soft_processor_access_(true)
  // ft_publisher_(NULL)
{

}
  
WG06::~WG06()
{
  if (pressure_publisher_) delete pressure_publisher_;
  if (accel_publisher_) delete accel_publisher_;
}

void WG06::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{ 
  WG0X::construct(sh, start_address);

  has_accel_and_ft_ = false;

  // As good a place as any for making sure that compiler actually packed these structures correctly
  BOOST_STATIC_ASSERT(sizeof(WG06StatusWithAccel) == WG06StatusWithAccel::SIZE);
  BOOST_STATIC_ASSERT(sizeof(FTDataSample) == FTDataSample::SIZE);
  BOOST_STATIC_ASSERT(sizeof(WG06Pressure) == WG06Pressure::SIZE);
  BOOST_STATIC_ASSERT(sizeof(WG06BigPressure) == WG06BigPressure::SIZE);
  BOOST_STATIC_ASSERT(sizeof(WG06StatusWithAccelAndFT) == WG06StatusWithAccelAndFT::SIZE);  

  unsigned int base_status_size = sizeof(WG0XStatus);

  command_size_ = sizeof(WG0XCommand);
  status_size_ = sizeof(WG0XStatus);
  pressure_size_ = sizeof(WG06Pressure);
  unsigned pressure_phy_addr = PRESSURE_PHY_ADDR;

  if (fw_major_ == 0)
  {
    // Do nothing - status memory map is same size as WG05
  }
  if (fw_major_ == 1)
  {
    // Include Accelerometer data
    status_size_ = base_status_size = sizeof(WG06StatusWithAccel);
  }
  else if ((fw_major_ == 2) || (fw_major_ == 3))
  {
    // Include Accelerometer and Force/Torque sensor data
    status_size_ = base_status_size = sizeof(WG06StatusWithAccelAndFT);
    has_accel_and_ft_ = true;
    if (fw_major_ == 3)
    {
      // Pressure data size is 513 bytes instead of 94.  The physical address has also moved.
      pressure_size_ = sizeof(WG06BigPressure);
      pressure_phy_addr = BIG_PRESSURE_PHY_ADDR;
    }
  }
  else 
  {
    ROS_ERROR("Unsupported WG06 FW major version %d", fw_major_);
  }
  status_size_ += pressure_size_;


  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(3);
  //ROS_DEBUG("device %d, command  0x%X = 0x10000+%d", (int)sh->get_ring_position(), start_address, start_address-0x10000);
  (*fmmu)[0] = EC_FMMU(start_address, // Logical start address
                       command_size_,// Logical length
                       0x00, // Logical StartBit
                       0x07, // Logical EndBit
                       COMMAND_PHY_ADDR, // Physical Start address
                       0x00, // Physical StartBit
                       false, // Read Enable
                       true, // Write Enable
                       true); // Enable

  start_address += command_size_;

  //ROS_DEBUG("device %d, status   0x%X = 0x10000+%d", (int)sh->get_ring_position(), start_address, start_address-0x10000);
  (*fmmu)[1] = EC_FMMU(start_address, // Logical start address
                       base_status_size, // Logical length
                       0x00, // Logical StartBit
                       0x07, // Logical EndBit
                       STATUS_PHY_ADDR, // Physical Start address
                       0x00, // Physical StartBit
                       true, // Read Enable
                       false, // Write Enable
                       true); // Enable

  start_address += base_status_size;

  (*fmmu)[2] = EC_FMMU(start_address, // Logical start address
                       pressure_size_, // Logical length
                       0x00, // Logical StartBit
                       0x07, // Logical EndBit
                       pressure_phy_addr, // Physical Start address
                       0x00, // Physical StartBit
                       true, // Read Enable
                       false, // Write Enable
                       true); // Enable

  start_address += pressure_size_;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(5);

  // Sync managers
  (*pd)[0] = EC_SyncMan(COMMAND_PHY_ADDR, command_size_, EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;

  (*pd)[1] = EC_SyncMan(STATUS_PHY_ADDR, base_status_size);
  (*pd)[1].ChannelEnable = true;

  (*pd)[2] = EC_SyncMan(WGMailbox::MBX_COMMAND_PHY_ADDR, WGMailbox::MBX_COMMAND_SIZE, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
  (*pd)[2].ChannelEnable = true;
  (*pd)[2].ALEventEnable = true;

  (*pd)[3] = EC_SyncMan(WGMailbox::MBX_STATUS_PHY_ADDR, WGMailbox::MBX_STATUS_SIZE, EC_QUEUED);
  (*pd)[3].ChannelEnable = true;

  (*pd)[4] = EC_SyncMan(pressure_phy_addr, pressure_size_);
  (*pd)[4].ChannelEnable = true;

  sh->set_pd_config(pd);
}


int WG06::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  if ( ((fw_major_ == 1) && (fw_minor_ >= 1))  ||  (fw_major_ >= 2) )
  {
    app_ram_status_ = APP_RAM_PRESENT;
  }

  int retval = WG0X::initialize(hw, allow_unprogrammed);
  
  if (!retval && use_ros_)
  {
    bool poor_measured_motor_voltage = false;
    double max_pwm_ratio = double(0x2700) / double(PWM_MAX);
    double board_resistance = 5.0;
    if (!WG0X::initializeMotorModel(hw, "WG006", max_pwm_ratio, board_resistance, poor_measured_motor_voltage)) 
    {
      ROS_FATAL("Initializing motor trace failed");
      sleep(1); // wait for ros to flush rosconsole output
      return -1;
    }

    // For some versions of software pressure and force/torque sensors can be
    // selectively enabled / disabled    
    ros::NodeHandle nh(string("~/") + actuator_.name_);
    if (!nh.getParam("enable_pressure_sensor", enable_pressure_sensor_))
    {
      enable_pressure_sensor_ = true; //default to to true
    }
    if (!nh.getParam("enable_ft_sensor", enable_ft_sensor_))
    {
      enable_ft_sensor_ = false; //default to to false
    }

    if (enable_ft_sensor_ && (fw_major_ < 2))
    {
      ROS_WARN("Gripper firmware version %d does not support enabling force/torque sensor", fw_major_);
      enable_ft_sensor_ = false;
    }

    // FW version 2+ supports selectively enabling/disabling pressure and F/T sensor
    if (fw_major_ >= 2)
    {
      static const uint8_t PRESSURE_ENABLE_FLAG = 0x1;
      static const uint8_t FT_ENABLE_FLAG       = 0x2;
      static const unsigned PRESSURE_FT_ENABLE_ADDR = 0xAA;
      uint8_t pressure_ft_enable = 0;
      if (enable_pressure_sensor_) pressure_ft_enable |= PRESSURE_ENABLE_FLAG;
      if (enable_ft_sensor_) pressure_ft_enable |= FT_ENABLE_FLAG;
      EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
      if (writeMailbox(&com, PRESSURE_FT_ENABLE_ADDR, &pressure_ft_enable, 1) != 0)
      {
        ROS_FATAL("Could not enable/disable pressure and force/torque sensors");
        return -1;
      }
    }

    if (!initializePressure(hw))
    {
      return -1;
    }

    // Publish accelerometer data as a ROS topic, if firmware is recent enough
    if (fw_major_ >= 1)
    {
      if (!initializeAccel(hw))
      {
        return -1;
      }
    }

    // FW version 2 supports Force/Torque sensor.
    // Provide Force/Torque data to controllers as an AnalogIn vector 
    if ((fw_major_ >= 2) && (enable_ft_sensor_))
    {
      if (!initializeFT(hw))
      {
        return -1;
      }
    }

    // FW version 2 and 3 uses soft-processors to control certain peripherals.
    // Allow the firmware on these soft-processors to be read/write through ROS service calls
    if ((fw_major_ >= 2) && enable_soft_processor_access_)
    {
      if (!initializeSoftProcessor())
      {
        return -1;
      }
    }


  }

  return retval;
}


bool WG06::initializePressure(pr2_hardware_interface::HardwareInterface *hw)
{
  // Publish pressure sensor data as a ROS topic
  string topic = "pressure";
  if (!actuator_.name_.empty())
    topic = topic + "/" + string(actuator_.name_);
  pressure_publisher_ = new realtime_tools::RealtimePublisher<pr2_msgs::PressureState>(ros::NodeHandle(), topic, 1);
  
  // Register pressure sensor with pr2_hardware_interface::HardwareInterface
  for (int i = 0; i < 2; ++i) 
  {
    pressure_sensors_[i].state_.data_.resize(22);
    pressure_sensors_[i].name_ = string(actuator_info_.name_) + string(i ? "r_finger_tip" : "l_finger_tip");
    if (hw && !hw->addPressureSensor(&pressure_sensors_[i]))
    {
      ROS_FATAL("A pressure sensor of the name '%s' already exists.  Device #%02d has a duplicate name", pressure_sensors_[i].name_.c_str(), sh_->get_ring_position());
      return false;
    }
  }

  return true;
}


bool WG06::initializeAccel(pr2_hardware_interface::HardwareInterface *hw)
{
  string topic = "accelerometer";
  if (!actuator_.name_.empty())
  {
    topic = topic + "/" + string(actuator_.name_);
  }
  accel_publisher_ = new realtime_tools::RealtimePublisher<pr2_msgs::AccelerometerState>(ros::NodeHandle(), topic, 1);
  
  // Register accelerometer with pr2_hardware_interface::HardwareInterface
  accelerometer_.name_ = actuator_info_.name_;
  if (hw && !hw->addAccelerometer(&accelerometer_))
  {
    ROS_FATAL("An accelerometer of the name '%s' already exists.  Device #%02d has a duplicate name", accelerometer_.name_.c_str(), sh_->get_ring_position());
    return false;
  }
  return true;
}


bool WG06::initializeFT(pr2_hardware_interface::HardwareInterface *hw)
{
  ft_raw_analog_in_.name_ = actuator_.name_ + "_ft_raw";
  if (hw && !hw->addAnalogIn(&ft_raw_analog_in_))
  {
    ROS_FATAL("An analog in of the name '%s' already exists.  Device #%02d has a duplicate name",
              ft_raw_analog_in_.name_.c_str(), sh_->get_ring_position());
    return false;
  }

  // FT provides 6 values : 3 Forces + 3 Torques
  ft_raw_analog_in_.state_.state_.resize(6); 
  // FT usually provides 3-4 new samples per cycle
  force_torque_.state_.samples_.reserve(4);
  force_torque_.state_.good_ = true;

  // For now publish RAW F/T values for engineering purposes.  In future this publisher may be disabled by default.
  std::string topic = "raw_ft";
  if (!actuator_.name_.empty())
    topic = topic + "/" + string(actuator_.name_);
  raw_ft_publisher_ = new realtime_tools::RealtimePublisher<ethercat_hardware::RawFTData>(ros::NodeHandle(), topic, 1);
  if (raw_ft_publisher_ == NULL)
  {
    ROS_FATAL("Could not allocate raw_ft publisher");
    return false;
  }
  // Allocate space for raw f/t data values
  raw_ft_publisher_->msg_.samples.reserve(MAX_FT_SAMPLES);

  force_torque_.command_.halt_on_error_ = false;
  force_torque_.state_.good_ = true;

  if (!actuator_.name_.empty())
  {
    ft_analog_in_.state_.state_.resize(6);
    ros::NodeHandle nh("~" + string(actuator_.name_));
    FTParamsInternal ft_params;
    if ( ft_params.getRosParams(nh) )
    {
      ft_params_ = ft_params;
      ft_params_.print();
      // If we have ft_params, publish F/T values.  
      topic = "ft";
      if (!actuator_.name_.empty())
        topic = topic + "/" + string(actuator_.name_);
      ft_publisher_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(ros::NodeHandle(), topic, 1);
      if (ft_publisher_ == NULL)
      {
        ROS_FATAL("Could not allocate ft publisher");
        return false;
      }

      // Register force/torque sensor with pr2_hardware_interface::HardwareInterface
      force_torque_.name_ = actuator_.name_;
      if (hw && !hw->addForceTorque(&force_torque_))
      {
        ROS_FATAL("A force/torque sensor of the name '%s' already exists.  Device #%02d has a duplicate name", force_torque_.name_.c_str(), sh_->get_ring_position());
        return false;
      }
    }
  }

  return true;
}


bool WG06::initializeSoftProcessor()
{
  // TODO: do not use direct access to device.  Not safe while realtime loop is running
  // ALSO, this leaks memory
  EthercatDirectCom *com = new EthercatDirectCom(EtherCAT_DataLinkLayer::instance());

  // Add soft-processors to list
  soft_processor_.add(&mailbox_, actuator_.name_, "pressure", 0xA000, 0x249);
  soft_processor_.add(&mailbox_, actuator_.name_, "accel", 0xB000, 0x24A);

  // Start services
  if (!soft_processor_.initialize(com))
  {
    return false;
  }

  return true;
}



void WG06::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  if (reset) 
  {
    pressure_checksum_error_ = false;
    ft_overload_flags_ = 0;
    ft_disconnected_ = false;
    ft_vhalf_error_ = false;
    ft_sampling_rate_error_ = false;
  }

  WG0X::packCommand(buffer, halt, reset);

  WG0XCommand *c = (WG0XCommand *)buffer;

  if (accelerometer_.command_.range_ > 2 || 
      accelerometer_.command_.range_ < 0)
    accelerometer_.command_.range_ = 0;

  if (accelerometer_.command_.bandwidth_ > 6 || 
      accelerometer_.command_.bandwidth_ < 0)
    accelerometer_.command_.bandwidth_ = 0;
  
  c->digital_out_ = (digital_out_.command_.data_ != 0) |
    ((accelerometer_.command_.bandwidth_ & 0x7) << 1) | 
    ((accelerometer_.command_.range_ & 0x3) << 4); 
  c->checksum_ = wg_util::rotateRight8(wg_util::computeChecksum(c, command_size_ - 1));
}


bool WG06::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  bool rv = true;

  int status_bytes = 
    has_accel_and_ft_  ? sizeof(WG06StatusWithAccelAndFT) :  // Has FT sensor and accelerometer
    accel_publisher_   ? sizeof(WG06StatusWithAccel) : 
                         sizeof(WG0XStatus);  

  unsigned char *pressure_buf = (this_buffer + command_size_ + status_bytes);

  unsigned char* this_status = this_buffer + command_size_;
  if (!verifyChecksum(this_status, status_bytes))
  {
    status_checksum_error_ = true;
    rv = false;
    goto end;
  }

  if (!unpackPressure(pressure_buf))
  {
    rv = false;
    //goto end;  // all other tasks should not be effected by bad pressure sensor data
  }

  if (accel_publisher_)
  {
    WG06StatusWithAccel *status = (WG06StatusWithAccel *)(this_buffer + command_size_);
    WG06StatusWithAccel *last_status = (WG06StatusWithAccel *)(prev_buffer + command_size_);
    if (!unpackAccel(status, last_status))
    {
      rv=false;
    }
  }

  if (has_accel_and_ft_ && enable_ft_sensor_)
  {
    WG06StatusWithAccelAndFT *status = (WG06StatusWithAccelAndFT *)(this_buffer + command_size_);
    WG06StatusWithAccelAndFT *last_status = (WG06StatusWithAccelAndFT *)(prev_buffer + command_size_);
    if (!unpackFT(status, last_status))
    {
      rv = false;
    }
  }


  if (!WG0X::unpackState(this_buffer, prev_buffer))
  {
    rv = false;
  }

 end:
  return rv;
}


/*!
 * \brief Unpack pressure sensor samples from realtime data.
 *
 * \return True, if there are no problems, false if there is something wrong with the data. 
 */
bool WG06::unpackPressure(unsigned char *pressure_buf)
{  
  if (!enable_pressure_sensor_)
  {
    // If pressure sensor is not enabled don't attempt to do anything with pressure data
    return true;
  }

  if (!verifyChecksum(pressure_buf, pressure_size_))
  {
    ++pressure_checksum_error_count_;
    if (false /* debugging */)
    {
      std::stringstream ss;
      ss << "Pressure buffer checksum error : " << std::endl;
      for (unsigned ii=0; ii<pressure_size_; ++ii)
      {
        ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
           << unsigned(pressure_buf[ii]) << " ";
        if ((ii%8) == 7) ss << std::endl;
      }
      ROS_ERROR_STREAM(ss.str());
      std::cerr << ss.str() << std::endl;
    }
    pressure_checksum_error_ = true;
    return false;
  }
  else 
  {
    WG06Pressure *p( (WG06Pressure *) pressure_buf);
    for (int i = 0; i < 22; ++i ) {
      pressure_sensors_[0].state_.data_[i] =
        ((p->l_finger_tip_[i] >> 8) & 0xff) |
        ((p->l_finger_tip_[i] << 8) & 0xff00);
      pressure_sensors_[1].state_.data_[i] =
        ((p->r_finger_tip_[i] >> 8) & 0xff) |
        ((p->r_finger_tip_[i] << 8) & 0xff00);
    }

    if (p->timestamp_ != last_pressure_time_)
    {
      if (pressure_publisher_ && pressure_publisher_->trylock())
      {
        pressure_publisher_->msg_.header.stamp = ros::Time::now();
        pressure_publisher_->msg_.l_finger_tip.resize(22);
        pressure_publisher_->msg_.r_finger_tip.resize(22);
        for (int i = 0; i < 22; ++i ) {
          pressure_publisher_->msg_.l_finger_tip[i] = pressure_sensors_[0].state_.data_[i];
          pressure_publisher_->msg_.r_finger_tip[i] = pressure_sensors_[1].state_.data_[i];
        }
        pressure_publisher_->unlockAndPublish();
      }
    }
    last_pressure_time_ = p->timestamp_;
  }

  return true;
}


/*!
 * \brief Unpack 3-axis accelerometer samples from realtime data.
 *
 * \return True, if there are no problems.
 */
bool WG06::unpackAccel(WG06StatusWithAccel *status, WG06StatusWithAccel *last_status)
{
  int count = uint8_t(status->accel_count_ - last_status->accel_count_);
  accelerometer_samples_ += count;
  // Only most recent 4 samples of accelerometer data is available in status data
  // 4 samples will be enough with realtime loop running at 1kHz and accelerometer running at 3kHz
  // If count is greater than 4, then some data has been "missed".
  accelerometer_missed_samples_ += (count > 4) ? (count-4) : 0; 
  count = min(4, count);
  accelerometer_.state_.samples_.resize(count);
  accelerometer_.state_.frame_id_ = string(actuator_info_.name_) + "_accelerometer_link";
  for (int i = 0; i < count; ++i)
  {
    int32_t acc = status->accel_[count - i - 1];
    int range = (acc >> 30) & 3;
    float d = 1 << (8 - range);
    accelerometer_.state_.samples_[i].x = 9.81 * ((((acc >>  0) & 0x3ff) << 22) >> 22) / d;
    accelerometer_.state_.samples_[i].y = 9.81 * ((((acc >> 10) & 0x3ff) << 22) >> 22) / d;
    accelerometer_.state_.samples_[i].z = 9.81 * ((((acc >> 20) & 0x3ff) << 22) >> 22) / d;
  }

  if (accel_publisher_->trylock())
  {
    accel_publisher_->msg_.header.frame_id = accelerometer_.state_.frame_id_;
    accel_publisher_->msg_.header.stamp = ros::Time::now();
    accel_publisher_->msg_.samples.resize(count);
    for (int i = 0; i < count; ++i)
    {
      accel_publisher_->msg_.samples[i].x = accelerometer_.state_.samples_[i].x;
      accel_publisher_->msg_.samples[i].y = accelerometer_.state_.samples_[i].y;
      accel_publisher_->msg_.samples[i].z = accelerometer_.state_.samples_[i].z;
    }
    accel_publisher_->unlockAndPublish();
  }
  return true;
}


/*!
 * \brief Convert FTDataSample to Wrench using gain, offset, and coefficient matrix
 * 
 * perform offset and gains multiplication on raw data
 * and then multiply by calibration matrix to get force and torque values.
 * The calibration matrix is based on "raw"  deltaR/R values from strain gauges
 *
 * Force/Torque = Coeff * ADCVoltage
 *
 * Coeff = RawCoeff / ( ExcitationVoltage * AmplifierGain )
 *       = RawCoeff / ( 2.5V * AmplifierGain )
 *
 * ADCVoltage = Vref / 2^16 
 *            = 2.5 / 2^16
 * 
 * Force/Torque =  RawCalibrationCoeff / ( ExcitationVoltage * AmplifierGain ) * (ADCValues * 2.5V/2^16)
 *              = (RawCalibration * ADCValues) / (AmplifierGain * 2^16)
 * 
 * Note on hardware circuit and Vref and excitation voltage should have save value.  
 * Thus, with Force/Torque calculation they cancel out. 
 */
void WG06::convertFTDataSampleToWrench(const FTDataSample &sample, geometry_msgs::Wrench &wrench)
{
  // Apply gains / offset to F/T raw analog inputs
  // Also, make sure values are within bounds.  
  // Out-of-bound values indicate a broken sensor or an overload.
  double in[6];
  for (unsigned i=0; i<6; ++i)
  {
    int raw_data = sample.data_[i];
    if (abs(raw_data) > ft_overload_limit_)
    {
      ft_overload_flags_ |= (1<<i);
    }
    in[i] = (double(raw_data) - ft_params_.offset(i)) / ( ft_params_.gain(i) * double(1<<16) );
  }

  // Vhalf ADC measurement should be amost half the ADC reference voltage
  // For a 16bit ADC the Vhalf value should be about (1<<16)/2
  // If Vhalf value is not close to this, this could mean 1 of 2 things:
  //   1. WG035 electronics are damaged some-how 
  //   2. WG035 is not present or disconnected gripper MCB 
  if ( abs( int(sample.vhalf_) - FT_VHALF_IDEAL) > FT_VHALF_RANGE )
  {
    if ((sample.vhalf_ == 0x0000) || (sample.vhalf_ == 0xFFFF))
    {
      // When WG035 MCB is not present the DATA line floats low or high and all 
      // reads through SPI interface return 0.
      ft_disconnected_ = true;
    }
    else 
    {
      ft_vhalf_error_ = true;
    }
  }

  // Apply coeffiecient matrix multiplication to raw inputs to get force/torque values
  double out[6];
  for (unsigned i=0; i<6; ++i)
  {
    double sum=0.0;
    for (unsigned j=0; j<6; ++j)
    {
      sum += ft_params_.calibration_coeff(i,j) * in[j];
    }
    out[i] = sum;
  }

  wrench.force.x  = out[0];
  wrench.force.y  = out[1];
  wrench.force.z  = out[2];
  wrench.torque.x = out[3];
  wrench.torque.y = out[4];
  wrench.torque.z = out[5];
}


/*!
 * \brief Unpack force/torque ADC samples from realtime data.
 *
 * \return True, if there are no problems, false if there is something wrong with the data. 
 */
bool WG06::unpackFT(WG06StatusWithAccelAndFT *status, WG06StatusWithAccelAndFT *last_status)
{  
  pr2_hardware_interface::ForceTorqueState &ft_state(force_torque_.state_);

  ros::Time current_time(ros::Time::now());

  // Fill in raw analog output with most recent data sample, (might become deprecated?)
  {
    ft_raw_analog_in_.state_.state_.resize(6);
    const FTDataSample &sample(status->ft_samples_[0]);
    for (unsigned i=0; i<6; ++i)
    {
      int raw_data = sample.data_[i];
      ft_raw_analog_in_.state_.state_[i] = double(raw_data);
    }
  }

  unsigned new_samples = (unsigned(status->ft_sample_count_) - unsigned(last_status->ft_sample_count_)) & 0xFF;
  ft_sample_count_ += new_samples;
  int missed_samples = std::max(int(0), int(new_samples) - int(MAX_FT_SAMPLES));
  ft_missed_samples_ += missed_samples;
  unsigned usable_samples = min(new_samples, MAX_FT_SAMPLES); 

  // Also, if number of new_samples is ever 0, then there is also an error
  if (usable_samples == 0)
  {
    ft_sampling_rate_error_ = true;
  }

  // Make room in data structure for more f/t samples
  ft_state.samples_.resize(usable_samples);

  // add side "l" or "r" to frame_id
  string ft_link_id = string(actuator_info_.name_).substr(0,1) + "_force_torque_link";

  // If any f/t channel is overload or the sampling rate is bad, there is an error.
  ft_state.good_ = ( (!ft_sampling_rate_error_) && 
                     (ft_overload_flags_ == 0) && 
                     (!ft_disconnected_) && 
                     (!ft_vhalf_error_) );

  for (unsigned sample_index=0; sample_index<usable_samples; ++sample_index)
  {
    // samples are stored in status data, so that newest sample is at index 0.
    // this is the reverse of the order data is stored in hardware_interface::ForceTorque buffer.
    unsigned status_sample_index = usable_samples-sample_index-1;
    const FTDataSample &sample(status->ft_samples_[status_sample_index]); 
    geometry_msgs::Wrench &wrench(ft_state.samples_[sample_index]);
    convertFTDataSampleToWrench(sample, wrench);
  }

  // Put newest sample into analog vector for controllers (deprecated)
  if (usable_samples > 0)
  {
    const geometry_msgs::Wrench &wrench(ft_state.samples_[usable_samples-1]);
    ft_analog_in_.state_.state_[0] = wrench.force.x;
    ft_analog_in_.state_.state_[1] = wrench.force.y;
    ft_analog_in_.state_.state_[2] = wrench.force.z;
    ft_analog_in_.state_.state_[3] = wrench.torque.x;
    ft_analog_in_.state_.state_[4] = wrench.torque.y;
    ft_analog_in_.state_.state_[5] = wrench.torque.z;
  }

  // Put all new samples in buffer and publish it
  if ((raw_ft_publisher_ != NULL) && (raw_ft_publisher_->trylock()))
  {
    raw_ft_publisher_->msg_.samples.resize(usable_samples);
    raw_ft_publisher_->msg_.sample_count = ft_sample_count_;
    raw_ft_publisher_->msg_.missed_samples = ft_missed_samples_;
    for (unsigned sample_num=0; sample_num<usable_samples; ++sample_num)
    {
      // put data into message so oldest data is first element
      const FTDataSample &sample(status->ft_samples_[sample_num]);
      ethercat_hardware::RawFTDataSample &msg_sample(raw_ft_publisher_->msg_.samples[usable_samples-sample_num-1]);
      msg_sample.sample_count = ft_sample_count_ - sample_num;
      msg_sample.data.resize(NUM_FT_CHANNELS);
      for (unsigned ch_num=0; ch_num<NUM_FT_CHANNELS; ++ch_num)
      {
        msg_sample.data[ch_num] = sample.data_[ch_num];
      }
      msg_sample.vhalf = sample.vhalf_;
    }
    raw_ft_publisher_->msg_.sample_count = ft_sample_count_;
    raw_ft_publisher_->unlockAndPublish();
  }

  // Put newest sample in realtime publisher
  if ( (usable_samples > 0) && (ft_publisher_ != NULL) && (ft_publisher_->trylock()) )
  {
    ft_publisher_->msg_.header.stamp = current_time;
    ft_publisher_->msg_.header.frame_id = ft_link_id;
    ft_publisher_->msg_.wrench = ft_state.samples_[usable_samples-1];
    ft_publisher_->unlockAndPublish();
  }

  // If this returns false, it will cause motors to halt.
  // Return "good_" state of sensor, unless halt_on_error is false. 
  return ft_state.good_ || !force_torque_.command_.halt_on_error_;
}


void WG06::multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer)
{
  diagnostic_updater::DiagnosticStatusWrapper &d(diagnostic_status_);
  diagnosticsWG06(d, buffer);  
  vec.push_back(d);
  diagnosticsAccel(d, buffer);
  vec.push_back(d);  
  diagnosticsPressure(d, buffer);
  vec.push_back(d);

  if (has_accel_and_ft_ && enable_ft_sensor_)
  {
    WG06StatusWithAccelAndFT *status = (WG06StatusWithAccelAndFT *)(buffer + command_size_);
    // perform f/t sample
    diagnosticsFT(d, status);
    vec.push_back(d);
  }

  last_publish_time_ = ros::Time::now();
  first_publish_ = false;
}


void WG06::diagnosticsAccel(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  stringstream str;
  str << "Accelerometer (" << actuator_info_.name_ << ")";
  d.name = str.str();
  char serial[32];
  snprintf(serial, sizeof(serial), "%d-%05d-%05d", config_info_.product_id_ / 100000 , config_info_.product_id_ % 100000, config_info_.device_serial_number_);
  d.hardware_id = serial;

  d.summary(d.OK, "OK");
  d.clear();
  
  pr2_hardware_interface::AccelerometerCommand acmd(accelerometer_.command_);

  const char * range_str = 
    (acmd.range_ == 0) ? "+/-2G" :
    (acmd.range_ == 1) ? "+/-4G" :
    (acmd.range_ == 2) ? "+/-8G" :
    "INVALID";

  const char * bandwidth_str = 
    (acmd.bandwidth_ == 6) ? "1500Hz" :
    (acmd.bandwidth_ == 5)  ? "750Hz" :
    (acmd.bandwidth_ == 4)  ? "375Hz" :
    (acmd.bandwidth_ == 3)  ? "190Hz" :
    (acmd.bandwidth_ == 2)  ? "100Hz" :
    (acmd.bandwidth_ == 1)   ? "50Hz" :
    (acmd.bandwidth_ == 0)   ? "25Hz" :
    "INVALID";

  // Board revB=1 and revA=0 does not have accelerometer
  bool has_accelerometer = (board_major_ >= 2);
  double sample_frequency = 0.0;
  ros::Time current_time(ros::Time::now());
  if (!first_publish_)
  {
    sample_frequency = double(accelerometer_samples_) / (current_time - last_publish_time_).toSec();
    {
      if (((sample_frequency < 2000) || (sample_frequency > 4000)) && has_accelerometer)
      {
        d.mergeSummary(d.WARN, "Bad accelerometer sampling frequency");
      }
    }
  }
  accelerometer_samples_ = 0;

  d.addf("Accelerometer", "%s", accelerometer_.state_.samples_.size() > 0 ? "Ok" : "Not Present");
  d.addf("Accelerometer range", "%s (%d)", range_str, acmd.range_);
  d.addf("Accelerometer bandwidth", "%s (%d)", bandwidth_str, acmd.bandwidth_);
  d.addf("Accelerometer sample frequency", "%f", sample_frequency);
  d.addf("Accelerometer missed samples", "%d", accelerometer_missed_samples_);                                   
}


void WG06::diagnosticsWG06(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  WG0X::diagnostics(d, buffer);
  // nothing else to do here 
}

void WG06::diagnosticsPressure(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  int status_bytes = 
    has_accel_and_ft_  ? sizeof(WG06StatusWithAccelAndFT) :  // Has FT sensor and accelerometer
    accel_publisher_   ? sizeof(WG06StatusWithAccel) : 
                         sizeof(WG0XStatus);
  WG06Pressure *pressure = (WG06Pressure *)(buffer + command_size_ + status_bytes);

  stringstream str;
  str << "Pressure sensors (" << actuator_info_.name_ << ")";
  d.name = str.str();
  char serial[32];
  snprintf(serial, sizeof(serial), "%d-%05d-%05d", config_info_.product_id_ / 100000 , config_info_.product_id_ % 100000, config_info_.device_serial_number_);
  d.hardware_id = serial;
  d.clear();

  if (enable_pressure_sensor_)
  {    
    d.summary(d.OK, "OK");
  }
  else
  {
    d.summary(d.OK, "Pressure sensor disabled by user");
  }

  if (pressure_checksum_error_)
  {
    d.mergeSummary(d.ERROR, "Checksum error on pressure data");
  }
    
  if (enable_pressure_sensor_)
  {
    // look at pressure sensor value to detect any damaged cables, or possibly missing sensor
    unsigned l_finger_good_count = 0;
    unsigned r_finger_good_count = 0;

    for (unsigned region_num=0; region_num<NUM_PRESSURE_REGIONS; ++region_num)
    {
      uint16_t l_data = pressure->l_finger_tip_[region_num];
      if ((l_data != 0xFFFF) && (l_data != 0x0000))
      {
        ++l_finger_good_count;
      }

      uint16_t r_data = pressure->r_finger_tip_[region_num];
      if ((r_data != 0xFFFF) && (r_data != 0x0000))
      {
        ++r_finger_good_count;
      }
    }
      
    // if no pressure sensor regions are return acceptable data, then its possible that the
    // pressure sensor is not connected at all.  This is just a warning, because on many robots
    // the pressure sensor may not be installed
    if ((l_finger_good_count == 0) && (r_finger_good_count == 0))
    {
      d.mergeSummary(d.WARN, "Pressure sensors may not be connected");
    }
    else 
    {
      // At least one pressure sensor seems to be present ...       
      if (l_finger_good_count == 0)
      {
        d.mergeSummary(d.WARN, "Sensor on left finger may not be connected");
      }
      else if (l_finger_good_count < NUM_PRESSURE_REGIONS)
      {
        d.mergeSummary(d.WARN, "Sensor on left finger may have damaged sensor regions");
      }

      if (r_finger_good_count == 0)
      {
        d.mergeSummary(d.WARN, "Sensor on right finger may not be connected");
      }
      else if (r_finger_good_count < NUM_PRESSURE_REGIONS)
      {
        d.mergeSummary(d.WARN, "Sensor on right finger may have damaged sensor regions");
      }
    } 

    d.addf("Timestamp", "%u", pressure->timestamp_);
    d.addf("Data size", "%u", pressure_size_);
    d.addf("Checksum error count", "%u", pressure_checksum_error_count_);

    { // put right and left finger data in dianostics
      std::stringstream ss;

      for (unsigned region_num=0; region_num<NUM_PRESSURE_REGIONS; ++region_num)
      {
        ss << std::uppercase << std::hex << std::setw(4) << std::setfill('0') 
           << pressure->r_finger_tip_[region_num] << " ";
        if (region_num%8 == 7) 
          ss << std::endl;      
      }
      d.add("Right finger data",  ss.str());

      ss.str("");

      for (unsigned region_num=0; region_num<NUM_PRESSURE_REGIONS; ++region_num)
      {
        ss << std::uppercase << std::hex << std::setw(4) << std::setfill('0') 
           << pressure->l_finger_tip_[region_num] << " ";
        if (region_num%8 == 7) ss << std::endl;      
      }
      d.add("Left finger data",  ss.str());
    }
  }

}


void WG06::diagnosticsFT(diagnostic_updater::DiagnosticStatusWrapper &d, WG06StatusWithAccelAndFT *status)
{
  stringstream str;
  str << "Force/Torque sensor (" << actuator_info_.name_ << ")";
  d.name = str.str();
  char serial[32];
  snprintf(serial, sizeof(serial), "%d-%05d-%05d", config_info_.product_id_ / 100000 , config_info_.product_id_ % 100000, config_info_.device_serial_number_);
  d.hardware_id = serial;

  d.summary(d.OK, "OK");
  d.clear();

  ros::Time current_time(ros::Time::now());
  double sample_frequency = 0.0;
  if (!first_publish_)
  {
    sample_frequency = double(ft_sample_count_ - diag_last_ft_sample_count_) / (current_time - last_publish_time_).toSec();
  }
  diag_last_ft_sample_count_ = ft_sample_count_;

  //d.addf("F/T sample count", "%llu", ft_sample_count_);
  d.addf("F/T sample frequency", "%.2f (Hz)", sample_frequency);
  d.addf("F/T missed samples", "%llu", ft_missed_samples_);
  std::stringstream ss;
  const FTDataSample &sample(status->ft_samples_[0]);  //use newest data sample
  for (unsigned i=0;i<NUM_FT_CHANNELS;++i)
  {
    ss.str(""); ss << "Ch"<< (i);
    d.addf(ss.str(), "%d", int(sample.data_[i]));
  }
  d.addf("FT Vhalf", "%d", int(sample.vhalf_));

  if (ft_overload_flags_ != 0)
  {
    d.mergeSummary(d.ERROR, "Sensor overloaded");
    ss.str("");
    for (unsigned i=0;i<NUM_FT_CHANNELS;++i)
    {
      ss << "Ch" << i << " ";
    }
  }
  else 
  {
    ss.str("None");
  }
  d.add("Overload Channels", ss.str());

  if (ft_sampling_rate_error_)
  {
    d.mergeSummary(d.ERROR, "Sampling rate error");
  }

  if (ft_disconnected_)
  {
    d.mergeSummary(d.ERROR, "Amplifier disconnected");
  }
  else if (ft_vhalf_error_)
  {
    d.mergeSummary(d.ERROR, "Vhalf error, amplifier circuity may be damaged"); 
  }

  const std::vector<double> &ft_data( ft_analog_in_.state_.state_ );
  if (ft_data.size() == 6)
  {
    d.addf("Force X",  "%f", ft_data[0]);
    d.addf("Force Y",  "%f", ft_data[1]);
    d.addf("Force Z",  "%f", ft_data[2]);
    d.addf("Torque X", "%f", ft_data[3]);
    d.addf("Torque Y", "%f", ft_data[4]);
    d.addf("Torque Z", "%f", ft_data[5]);
  }
}



FTParamsInternal::FTParamsInternal()
{
  // Initial offset = 0.0
  // Gains = 1.0 
  // Calibration coeff = identity matrix
  for (int i=0; i<6; ++i)
  {
    offset(i) = 0.0;
    gain(i) = 1.0;
    for (int j=0; j<6; ++j)
    {
      calibration_coeff(i,j) = (i==j) ? 1.0 : 0.0;
    }
  }
}


void FTParamsInternal::print() const
{
  for (int i=0; i<6; ++i)
  {
    ROS_INFO("offset[%d] = %f", i, offset(i));
  }
  for (int i=0; i<6; ++i)
  {
    ROS_INFO("gain[%d] = %f", i, gain(i));
  }
  for (int i=0; i<6; ++i)
  {
    ROS_INFO("coeff[%d] = [%f,%f,%f,%f,%f,%f]", i, 
             calibration_coeff(i,0), calibration_coeff(i,1), 
             calibration_coeff(i,2), calibration_coeff(i,3), 
             calibration_coeff(i,4), calibration_coeff(i,5)
             );
  }
}


bool FTParamsInternal::getDoubleArray(XmlRpc::XmlRpcValue params, const char* name, double *results, unsigned len)
{
  if(!params.hasMember(name))
  {
    ROS_ERROR("Expected ft_param to have '%s' element", name);
    return false;
  }

  XmlRpc::XmlRpcValue values = params[name];
  if (values.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Expected FT param '%s' to be list type", name);
    return false;
  }
  if (values.size() != int(len))
  {
    ROS_ERROR("Expected FT param '%s' to have %d elements", name, len);
    return false;
  }
  for (unsigned i=0; i<len; ++i)
  {
    if (values[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_ERROR("Expected FT param %s[%d] to be floating point type", name, i);
      return false;
    } else {
      results[i] = static_cast<double>(values[i]);
    }
  }

  return true;
}


/*!
 * \brief Grabs ft rosparams from a given node hande namespace
 *
 * The force/torque parameters consist of 
 *  6x ADC offset values
 *  6x6 gain matrix as 6-elment array of 6-element arrays of doubles
 *
 * \return True, if there are no problems.
 */
bool FTParamsInternal::getRosParams(ros::NodeHandle nh)
{
  if (!nh.hasParam("ft_params"))
  {
    ROS_WARN("'ft_params' not available for force/torque sensor in namespace '%s'",
             nh.getNamespace().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue params;
  nh.getParam("ft_params", params);
  if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("expected ft_params to be struct type");
    return false;
  }

  if (!getDoubleArray(params, "offsets", offsets_, 6))
  {
    return false;
  }

  if (!getDoubleArray(params, "gains", gains_, 6))
  {
    return false;
  }

  XmlRpc::XmlRpcValue coeff_matrix = params["calibration_coeff"];
  if (coeff_matrix.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Expected FT param 'calibration_coeff' to be list type");
    return false;
  }
  if (coeff_matrix.size() != 6)
  {
    ROS_ERROR("Expected FT param 'calibration_coeff' to have 6 elements");
    return false;
  }

  for (int i=0; i<6; ++i)
  {
    XmlRpc::XmlRpcValue coeff_row = coeff_matrix[i];
    if (coeff_row.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected FT param calibration_coeff[%d] to be list type", i);
      return false;
    }
    if (coeff_row.size() != 6)
    {
      ROS_ERROR("Expected FT param calibration_coeff[%d] to have 6 elements", i);
      return false;
    }
    
    for (int j=0; j<6; ++j)
    {
      if (coeff_row[j].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      {
        ROS_ERROR("Expected FT param calibration_coeff[%d,%d] to be floating point type", i,j);
        return false;
      } else {
        calibration_coeff(i,j) = static_cast<double>(coeff_row[j]);
      }
    }
  }

  return true;
}


const unsigned WG06::NUM_FT_CHANNELS;
const unsigned WG06::MAX_FT_SAMPLES;
