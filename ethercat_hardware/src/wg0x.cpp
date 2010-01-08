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

#include <math.h>

#include <ethercat_hardware/wg0x.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <boost/crc.hpp>

PLUGINLIB_REGISTER_CLASS(6805005, WG05, EthercatDevice);
PLUGINLIB_REGISTER_CLASS(6805006, WG06, EthercatDevice);
PLUGINLIB_REGISTER_CLASS(6805021, WG021, EthercatDevice);


// Temporary,, need 'log' fuction that can switch between fprintf and ROS_LOG.
#define ERR_MODE "\033[41m"
#define STD_MODE "\033[0m"
#define WARN_MODE "\033[43m"
#define GOOD_MODE "\033[42m"
#define INFO_MODE "\033[44m"

#define ERROR_HDR "\033[41mERROR\033[0m"
#define WARN_HDR "\033[43mERROR\033[0m"


static unsigned int rotateRight8(unsigned in)
{
  in &= 0xff;
  in = (in >> 1) | (in << 7);
  in &= 0xff;
  return in;
}

static unsigned computeChecksum(void const *data, unsigned length)
{
  unsigned char *d = (unsigned char *)data;
  unsigned int checksum = 0x42;
  for (unsigned int i = 0; i < length; ++i)
  {
    checksum = rotateRight8(checksum);
    checksum ^= d[i];
    checksum &= 0xff;
  }
  return checksum;
}

bool WG0XMbxHdr::build(unsigned address, unsigned length, MbxCmdType type, unsigned seqnum)
{
  if (type==LOCAL_BUS_WRITE) 
  {
    if (length > MBX_DATA_SIZE) 
    {
      fprintf(stderr, "size of %d is too large for write\n", length);
      return false;
    }
  }
  else if (type==LOCAL_BUS_READ) 
  {
    // Result of mailbox read, only stores result data + 1byte checksum
    if (length > (MBX_SIZE-1))
    {
      fprintf(stderr, "size of %d is too large for read\n", length);
      return false;      
    }
  }
  else {
    assert(0 && "invalid MbxCmdType");
    return false;
  }
  
  address_ = address;
  length_ = length - 1;
  seqnum_ = seqnum;
  write_nread_ = (type==LOCAL_BUS_WRITE) ? 1 : 0;
  checksum_ = rotateRight8(computeChecksum(this, sizeof(*this) - 1));
  return true;
}

bool WG0XMbxHdr::verifyChecksum(void) const
{
  return computeChecksum(this, sizeof(*this)) != 0;
}

bool WG0XMbxCmd::build(unsigned address, unsigned length, MbxCmdType type, unsigned seqnum, void const* data)
{
  if (!this->hdr_.build(address, length, type, seqnum))
  {
    return false;
  }
      
  if (data != NULL)
  {
    memcpy(data_, data, length);
  }
  else
  {
    memset(data_, 0, length);
  }
  unsigned int checksum = rotateRight8(computeChecksum(data_, length));
  data_[length] = checksum;
  return true;
}


MbxDiagnostics::MbxDiagnostics() :
  write_errors_(0),
  read_errors_(0),
  lock_errors_(0),
  retries_(0),
  retry_errors_(0)
{
  // Empty
}

WG0XDiagnostics::WG0XDiagnostics() :
  valid_(false),
  safety_disable_total_(0),
  undervoltage_total_(0),
  over_current_total_(0),
  board_over_temp_total_(0),
  bridge_over_temp_total_(0),
  operate_disable_total_(0),
  watchdog_disable_total_(0)
{
  memset(&safety_disable_status_, 0, sizeof(safety_disable_status_));
  memset(&safety_disable_counters_, 0, sizeof(safety_disable_counters_));
}

/*!
 * \brief  Use new updates WG0X diagnostics with new safety disable data
 *
 * \param new_status    newly collected safety disable status 
 * \param new_counters  newly collected safety disable counters
 */
void WG0XDiagnostics::update(const WG0XSafetyDisableStatus &new_status, const WG0XSafetyDisableCounters &new_counters)
{
  safety_disable_total_   += 0xFF & ((uint32_t)(new_status.safety_disable_count_ - safety_disable_status_.safety_disable_count_));
  undervoltage_total_     += 0xFF & ((uint32_t)(new_counters.undervoltage_count_ - safety_disable_counters_.undervoltage_count_));
  over_current_total_     += 0xFF & ((uint32_t)(new_counters.over_current_count_ - safety_disable_counters_.over_current_count_));
  board_over_temp_total_  += 0xFF & ((uint32_t)(new_counters.board_over_temp_count_ - safety_disable_counters_.board_over_temp_count_));
  bridge_over_temp_total_ += 0xFF & ((uint32_t)(new_counters.bridge_over_temp_count_ - safety_disable_counters_.bridge_over_temp_count_));
  operate_disable_total_  += 0xFF & ((uint32_t)(new_counters.operate_disable_count_ - safety_disable_counters_.operate_disable_count_));
  watchdog_disable_total_ += 0xFF & ((uint32_t)(new_counters.watchdog_disable_count_ - safety_disable_counters_.watchdog_disable_count_));

  safety_disable_status_   = new_status;
  safety_disable_counters_ = new_counters;
}

WG0X::WG0X()
{
  int error;
  if ((error = pthread_mutex_init(&wg0x_diagnostics_lock_, NULL)) != 0)
  {
    fprintf(stderr, ERROR_HDR " : WG0X : init diagnostics mutex :%s\n", strerror(error));
  }
  if ((error = pthread_mutex_init(&mailbox_lock_, NULL)) != 0)
  {
    fprintf(stderr, ERROR_HDR " : WG0X : init mailbox mutex :%s\n", strerror(error));
  }
}

WG0X::~WG0X()
{
  delete sh_->get_fmmu_config();
  delete sh_->get_pd_config();
}
  
WG06::~WG06()
{
  if (pressure_publisher_) delete pressure_publisher_;
  if (accel_publisher_) delete accel_publisher_;
}

void WG0X::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  EthercatDevice::construct(sh, start_address);

  reason_ = "OK";
  level_ = 0;

  voltage_error_ = max_voltage_error_ = 0;
  filtered_voltage_error_ = max_filtered_voltage_error_ = 0;
  current_error_ = max_current_error_ = 0;
  filtered_current_error_ = max_filtered_current_error_ = 0;
  voltage_estimate_ = 0;
  last_timestamp_ = 0;
  last_last_timestamp_ = 0;
  drops_ = 0;
  consecutive_drops_ = 0;
  max_consecutive_drops_ = 0;
  max_board_temperature_ = 0;
  max_bridge_temperature_ = 0;
  in_lockout_ = false;

  fw_major_ = (sh->get_revision() >> 8) & 0xff;
  fw_minor_ = sh->get_revision() & 0xff;
  board_major_ = ((sh->get_revision() >> 24) & 0xff) - 1;
  board_minor_ = (sh->get_revision() >> 16) & 0xff;

  bool isWG06 = sh_->get_product_code() == WG06::PRODUCT_CODE;
  bool isWG021 = sh_->get_product_code() == WG021::PRODUCT_CODE;
  unsigned int base_status = sizeof(WG0XStatus);

  command_size_ = sizeof(WG0XCommand);
  status_size_ = sizeof(WG0XStatus);
  if (isWG06)
  {
    if (fw_major_ >= 1)
      status_size_ = base_status = sizeof(WG06StatusWithAccel);
    status_size_ += sizeof(WG06Pressure);
  }
  else if (isWG021)
  {
    status_size_ = base_status = sizeof(WG021Status);
    command_size_ = sizeof(WG021Command);
  }

  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(isWG06 ? 3 : 2);
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

  (*fmmu)[1] = EC_FMMU(start_address, // Logical start address
                       base_status, // Logical length
                       0x00, // Logical StartBit
                       0x07, // Logical EndBit
                       STATUS_PHY_ADDR, // Physical Start address
                       0x00, // Physical StartBit
                       true, // Read Enable
                       false, // Write Enable
                       true); // Enable

  start_address += base_status;

  if (isWG06)
  {
    (*fmmu)[2] = EC_FMMU(start_address, // Logical start address
                         sizeof(WG06Pressure), // Logical length
                         0x00, // Logical StartBit
                         0x07, // Logical EndBit
                         PRESSURE_PHY_ADDR, // Physical Start address
                         0x00, // Physical StartBit
                         true, // Read Enable
                         false, // Write Enable
                         true); // Enable

    start_address += sizeof(WG06Pressure);
  }
  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(isWG06 ? 5 : 4);

  // Sync managers
  (*pd)[0] = EC_SyncMan(COMMAND_PHY_ADDR, command_size_, EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;

  (*pd)[1] = EC_SyncMan(STATUS_PHY_ADDR, base_status);
  (*pd)[1].ChannelEnable = true;

  (*pd)[2] = EC_SyncMan(MBX_COMMAND_PHY_ADDR, MBX_COMMAND_SIZE, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
  (*pd)[2].ChannelEnable = true;
  (*pd)[2].ALEventEnable = true;

  (*pd)[3] = EC_SyncMan(MBX_STATUS_PHY_ADDR, MBX_STATUS_SIZE, EC_QUEUED);
  (*pd)[3].ChannelEnable = true;

  if (isWG06)
  {
    (*pd)[4] = EC_SyncMan(PRESSURE_PHY_ADDR, sizeof(WG06Pressure));
    (*pd)[4].ChannelEnable = true;
  }

  sh->set_pd_config(pd);
}

int WG06::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  int retval = WG0X::initialize(hw, allow_unprogrammed);
  
  if (!retval && use_ros_)
  {
    // Publish pressure sensor data as a ROS topic
    string topic = "pressure";
    if (!actuator_.name_.empty())
      topic = topic + "/" + string(actuator_.name_);
    pressure_publisher_ = new realtime_tools::RealtimePublisher<pr2_msgs::PressureState>(ros::NodeHandle(), topic, 1);

    // Register accelerometer with pr2_hardware_interface::HardwareInterface
    for (int i = 0; i < 2; ++i) 
    {
      pressure_sensors_[i].state_.data_.resize(22);
      pressure_sensors_[i].name_ = string(actuator_info_.name_) + string(i ? "r_finger_tip" : "l_finger_tip");
      if (hw && !hw->addPressureSensor(&pressure_sensors_[i]))
      {
          ROS_FATAL("A pressure sensor of the name '%s' already exists.  Device #%02d has a duplicate name", pressure_sensors_[i].name_.c_str(), sh_->get_ring_position());
          ROS_BREAK();
          return -1;
      }
    }

    // Publish accelerometer data as a ROS topic, if firmware is recent enough
    if (fw_major_ >= 1)
    {
      topic = "/accelerometer/";
      if (!actuator_.name_.empty())
        topic += actuator_.name_;
      accel_publisher_ = new realtime_tools::RealtimePublisher<pr2_msgs::AccelerometerState>(ros::NodeHandle(), topic, 1);

      // Register accelerometer with pr2_hardware_interface::HardwareInterface
      {
        accelerometer_.name_ = actuator_info_.name_;
        if (hw && !hw->addAccelerometer(&accelerometer_))
        {
            ROS_FATAL("An accelerometer of the name '%s' already exists.  Device #%02d has a duplicate name", accelerometer_.name_.c_str(), sh_->get_ring_position());
            ROS_BREAK();
            return -1;
        }
      }

    }

  }

  return retval;
}

int WG021::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  int retval = WG0X::initialize(hw, allow_unprogrammed);

  // Register digital outs with pr2_hardware_interface::HardwareInterface
  struct {
    pr2_hardware_interface::DigitalOut *d;
    string name;
  } digital_outs[] = {
    {&digital_out_A_, "_digital_out_A"},
    {&digital_out_B_, "_digital_out_B"},
    {&digital_out_I_, "_digital_out_I"},
    {&digital_out_M_, "_digital_out_M"},
    {&digital_out_L0_, "_digital_out_L0"},
    {&digital_out_L1_, "_digital_out_L1"},
  };

  for (size_t i = 0; i < sizeof(digital_outs)/sizeof(digital_outs[0]); ++i)
  {
    digital_outs[i].d->name_ = string(actuator_info_.name_) + digital_outs[i].name;
    if (hw && !hw->addDigitalOut(digital_outs[i].d))
    {
        ROS_FATAL("A digital out of the name '%s' already exists.  Device #%02d has a duplicate name", digital_outs[i].d->name_.c_str(), sh_->get_ring_position());
        ROS_BREAK();
        return -1;
    }
  }

  // Register projector with pr2_hardware_interface::HardwareInterface
  {
    projector_.name_ = actuator_info_.name_;
    if (hw && !hw->addProjector(&projector_))
    {
        ROS_FATAL("A projector of the name '%s' already exists.  Device #%02d has a duplicate name", projector_.name_.c_str(), sh_->get_ring_position());
        ROS_BREAK();
        return -1;
    }
    projector_.command_.enable_ = true;
    projector_.command_.current_ = 0;
  }

  return retval;
}

int WG0X::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  ROS_DEBUG("Device #%02d: WG0%d (%#08x) Firmware Revision %d.%02d, PCB Revision %c.%02d, Serial #: %d", 
            sh_->get_ring_position(),
            sh_->get_product_code() % 100,
            sh_->get_product_code(), fw_major_, fw_minor_,
            'A' + board_major_, board_minor_,
            sh_->get_serial());

  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());

  if (sh_->get_product_code() == WG05::PRODUCT_CODE)
  {
    if (fw_major_ != 1 || fw_minor_ < 7)
    {
      ROS_FATAL("Unsupported firmware revision %d.%02d\n", fw_major_, fw_minor_);
      ROS_BREAK();
      return -1;
    }
  }
  else
  {
    if ((fw_major_ == 0 && fw_minor_ < 4) /*|| (fw_major_ == 1 && fw_minor_ < 0)*/)
    {
      ROS_FATAL("Unsupported firmware revision %d.%02d\n", fw_major_, fw_minor_);
      ROS_BREAK();
      return -1;
    }
  }

  if (readMailbox(&com, WG0XConfigInfo::CONFIG_INFO_BASE_ADDR, &config_info_, sizeof(config_info_)) != 0)
  {
    ROS_FATAL("Unable to load configuration information");
    ROS_BREAK();
    return -1;
  }
  ROS_DEBUG("            Serial #: %05d", config_info_.device_serial_number_);

  if (readEeprom(&com) < 0)
  {
    ROS_FATAL("Unable to read actuator info from EEPROM\n");
    ROS_BREAK();
    return -1;
  }

  boost::crc_32_type crc32;
  crc32.process_bytes(&actuator_info_, sizeof(actuator_info_)-sizeof(actuator_info_.crc32_));
  if (actuator_info_.crc32_ == crc32.checksum())
  {
    if (actuator_info_.major_ != 0 || actuator_info_.minor_ != 2)
    {
      if (allow_unprogrammed)
        ROS_WARN("Unsupported actuator info version (%d.%d != 0.2).  Please reprogram device #%02d", actuator_info_.major_, actuator_info_.minor_, sh_->get_ring_position());
      else
      {
        ROS_FATAL("Unsupported actuator info version (%d.%d != 0.2).  Please reprogram device #%02d", actuator_info_.major_, actuator_info_.minor_, sh_->get_ring_position());
        ROS_BREAK();
        return -1;
      }
    }

    actuator_.name_ = actuator_info_.name_;
    backemf_constant_ = 1.0 / (actuator_info_.speed_constant_ * 2 * M_PI * 1.0/60);
    ROS_DEBUG("            Name: %s", actuator_info_.name_);

    bool isWG021 = sh_->get_product_code() == WG021::PRODUCT_CODE;
    if (!isWG021)
    {
      // Register actuator with pr2_hardware_interface::HardwareInterface
      if (hw && !hw->addActuator(&actuator_))
      {
          ROS_FATAL("An actuator of the name '%s' already exists.  Device #%02d has a duplicate name", actuator_.name_.c_str(), sh_->get_ring_position());
          ROS_BREAK();
          return -1;
      }
    }

    // Register digital out with pr2_hardware_interface::HardwareInterface
    digital_out_.name_ = actuator_info_.name_;
    if (hw && !hw->addDigitalOut(&digital_out_))
    {
        ROS_FATAL("A digital out of the name '%s' already exists.  Device #%02d has a duplicate name", digital_out_.name_.c_str(), sh_->get_ring_position());
        ROS_BREAK();
        return -1;
    }
  }
  else if (allow_unprogrammed)
  {
    ROS_WARN("WARNING: Device #%02d (%d%05d) is not programmed", 
             sh_->get_ring_position(), sh_->get_product_code(), sh_->get_serial());
    actuator_info_.crc32_ = 0;
  }
  else
  {
    ROS_FATAL("Device #%02d (%d%05d) is not programmed, aborting...", 
              sh_->get_ring_position(), sh_->get_product_code(), sh_->get_serial());
    ROS_BREAK();
    return -1;
  }

  return 0;
}

#define GET_ATTR(a) \
{ \
  TiXmlElement *c; \
  attr = elt->Attribute((a)); \
  if (!attr) { \
    c = elt->FirstChildElement((a)); \
    if (!c || !(attr = c->GetText())) { \
      ROS_FATAL("Actuator is missing the attribute "#a); \
      ROS_BREAK(); \
    } \
  } \
}

void WG0X::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  pr2_hardware_interface::ActuatorCommand &cmd = actuator_.command_;

  // Override enable if motors are halted
  bool tmp = cmd.enable_;
  if (halt) {
    cmd.enable_ = reset;
    cmd.effort_ = 0;
  }

  if (reset) {
    level_ = 0;
    reason_ = "OK";
  }

  // Compute the current
  double current = (cmd.effort_ / actuator_info_.encoder_reduction_) / actuator_info_.motor_torque_constant_ ;
  actuator_.state_.last_commanded_effort_ = cmd.effort_;
  actuator_.state_.last_commanded_current_ = current;

  // Truncate the current to limit
  current = max(min(current, actuator_info_.max_current_), -actuator_info_.max_current_);

  // Pack command structures into EtherCAT buffer
  WG0XCommand *c = (WG0XCommand *)buffer;
  memset(c, 0, command_size_);
  c->programmed_current_ = int(current / config_info_.nominal_current_scale_);
  c->mode_ = cmd.enable_ ? (MODE_ENABLE | MODE_CURRENT | MODE_SAFETY_RESET) : MODE_OFF;
  c->digital_out_ = digital_out_.command_.data_;
  c->checksum_ = rotateRight8(computeChecksum(c, command_size_ - 1));

  // Restore enable
  cmd.enable_ = tmp;
}

void WG06::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  WG0X::packCommand(buffer, halt, reset);

  WG0XCommand *c = (WG0XCommand *)buffer;

  if (accelerometer_.command_.range_ > 2 || 
      accelerometer_.command_.range_ < 0)
    accelerometer_.command_.range_ = 0;

  c->digital_out_ = (digital_out_.command_.data_ != 0) |
    ((accelerometer_.command_.bandwidth_ & 0x7) << 1) | 
    ((accelerometer_.command_.range_ & 0x3) << 4); 
  c->checksum_ = rotateRight8(computeChecksum(c, command_size_ - 1));
}

void WG021::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  pr2_hardware_interface::ProjectorCommand &cmd = projector_.command_;

  // Override enable if motors are halted
  bool tmp = cmd.enable_;
  if (halt) {
    cmd.enable_ = reset;
    cmd.current_ = 0;
  }
  
  if (reset) {
    level_ = 0;
    reason_ = "OK";
  }

  // Truncate the current to limit
  projector_.state_.last_commanded_current_ = cmd.current_;
  cmd.current_ = max(min(cmd.current_, actuator_info_.max_current_), -actuator_info_.max_current_);

  // Pack command structures into EtherCAT buffer
  WG021Command *c = (WG021Command *)buffer;
  memset(c, 0, command_size_);
  c->digital_out_ = digital_out_.command_.data_;
  c->programmed_current_ = int(cmd.current_ / config_info_.nominal_current_scale_);
  c->mode_ = cmd.enable_ ? (MODE_ENABLE | MODE_CURRENT | MODE_SAFETY_RESET) : MODE_OFF;
  c->config0_ = ((cmd.A_ & 0xf) << 4) | ((cmd.B_ & 0xf) << 0);
  c->config1_ = ((cmd.I_ & 0xf) << 4) | ((cmd.M_ & 0xf) << 0);
  c->config2_ = ((cmd.L0_ & 0xf) << 4) | ((cmd.L1_ & 0xf) << 0);
  c->general_config_ = cmd.pulse_replicator_ == true;
  c->checksum_ = rotateRight8(computeChecksum(c, command_size_ - 1));

  // Restore enable
  cmd.enable_ = tmp;
}

bool WG06::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  int status_bytes = accel_publisher_ ? sizeof(WG06StatusWithAccel) : sizeof(WG0XStatus);

  WG06Pressure *p = (WG06Pressure *)(this_buffer + command_size_ + status_bytes);

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
      pressure_publisher_->msg_.set_l_finger_tip_size(22);
      pressure_publisher_->msg_.set_r_finger_tip_size(22);
      for (int i = 0; i < 22; ++i ) {
        pressure_publisher_->msg_.l_finger_tip[i] = pressure_sensors_[0].state_.data_[i];
        pressure_publisher_->msg_.r_finger_tip[i] = pressure_sensors_[1].state_.data_[i];
      }
      pressure_publisher_->unlockAndPublish();
    }
  }
  last_pressure_time_ = p->timestamp_;


  if (accel_publisher_)
  {
    WG06StatusWithAccel *status = (WG06StatusWithAccel *)(this_buffer + command_size_);
    WG06StatusWithAccel *last_status = (WG06StatusWithAccel *)(prev_buffer + command_size_);
    int count = min(uint8_t(4), uint8_t(status->accel_count_ - last_status->accel_count_));
    accelerometer_.state_.samples_.resize(count);
    accelerometer_.state_.frame_id_ = string(actuator_info_.name_) + "_accelerometer_link";
    for (int i = 0; i < count; ++i)
    {
      int32_t acc = status->accel_[count - i - 1];
      int range = (acc >> 30) & 3;
      float d = 1 << (8 - range);
      accelerometer_.state_.samples_[i].x = ((((acc >>  0) & 0x3ff) << 22) >> 22) / d;
      accelerometer_.state_.samples_[i].y = ((((acc >> 10) & 0x3ff) << 22) >> 22) / d;
      accelerometer_.state_.samples_[i].z = ((((acc >> 20) & 0x3ff) << 22) >> 22) / d;
    }

    if (accel_publisher_->trylock())
    {
      accel_publisher_->msg_.header.frame_id = accelerometer_.state_.frame_id_;
      accel_publisher_->msg_.header.stamp = ros::Time::now();
      accel_publisher_->msg_.set_samples_size(count);
      for (int i = 0; i < count; ++i)
      {
        accel_publisher_->msg_.samples[i].x = accelerometer_.state_.samples_[i].x;
        accel_publisher_->msg_.samples[i].y = accelerometer_.state_.samples_[i].y;
        accel_publisher_->msg_.samples[i].z = accelerometer_.state_.samples_[i].z;
      }
      accel_publisher_->unlockAndPublish();
    }
  }

  return WG0X::unpackState(this_buffer, prev_buffer);
}

bool WG0X::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  pr2_hardware_interface::ActuatorState &state = actuator_.state_;
  WG0XStatus *this_status, *prev_status;

  this_status = (WG0XStatus *)(this_buffer + command_size_);
  prev_status = (WG0XStatus *)(prev_buffer + command_size_);

  digital_out_.state_.data_ = this_status->digital_out_;

  state.timestamp_ = this_status->timestamp_ / 1e+6;
  state.device_id_ = sh_->get_ring_position();
  state.encoder_count_ = this_status->encoder_count_;
  state.position_ = double(this_status->encoder_count_) / actuator_info_.pulses_per_revolution_ * 2 * M_PI - state.zero_offset_;
  state.encoder_velocity_ = double(int(this_status->encoder_count_ - prev_status->encoder_count_))
      / (this_status->timestamp_ - prev_status->timestamp_) * 1e+6;
  state.velocity_ = state.encoder_velocity_ / actuator_info_.pulses_per_revolution_ * 2 * M_PI;
  state.calibration_reading_ = this_status->calibration_reading_ & LIMIT_SENSOR_0_STATE;
  state.calibration_rising_edge_valid_ = this_status->calibration_reading_ &  LIMIT_OFF_TO_ON;
  state.calibration_falling_edge_valid_ = this_status->calibration_reading_ &  LIMIT_ON_TO_OFF;
  state.last_calibration_rising_edge_ = double(this_status->last_calibration_rising_edge_) / actuator_info_.pulses_per_revolution_ * 2 * M_PI;
  state.last_calibration_falling_edge_ = double(this_status->last_calibration_falling_edge_) / actuator_info_.pulses_per_revolution_ * 2 * M_PI;
  state.is_enabled_ = this_status->mode_ != MODE_OFF;

  state.last_executed_current_ = this_status->programmed_current_ * config_info_.nominal_current_scale_;
  state.last_measured_current_ = this_status->measured_current_ * config_info_.nominal_current_scale_;

  state.last_executed_effort_ = this_status->programmed_current_ * config_info_.nominal_current_scale_ * actuator_info_.motor_torque_constant_ * actuator_info_.encoder_reduction_;
  state.last_measured_effort_ = this_status->measured_current_ * config_info_.nominal_current_scale_ * actuator_info_.motor_torque_constant_ * actuator_info_.encoder_reduction_;

  state.num_encoder_errors_ = this_status->num_encoder_errors_;

  state.motor_voltage_ = this_status->motor_voltage_ * config_info_.nominal_voltage_scale_;

  return verifyState(this_status, prev_status);
}

bool WG0X::verifyState(WG0XStatus *this_status, WG0XStatus *prev_status)
{
  pr2_hardware_interface::ActuatorState &state = actuator_.state_;
  bool rv = true;
  double expected_voltage;
  int level = 0;
  string reason = "OK";

  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());

  if (!(state.is_enabled_)) {
    goto end;
  }

  max_board_temperature_ = max(max_board_temperature_, this_status->board_temperature_);
  max_bridge_temperature_ = max(max_bridge_temperature_, this_status->bridge_temperature_);

  expected_voltage = state.last_measured_current_ * actuator_info_.resistance_ + state.velocity_ * actuator_info_.encoder_reduction_ * backemf_constant_;

  if (this_status->timestamp_ == last_timestamp_ ||
      this_status->timestamp_ == last_last_timestamp_) {
    ++drops_;
    ++consecutive_drops_;
    max_consecutive_drops_ = max(max_consecutive_drops_, consecutive_drops_);
  } else {
    consecutive_drops_ = 0;
  }
  last_last_timestamp_ = last_timestamp_;
  last_timestamp_ = this_status->timestamp_;

  if (consecutive_drops_ > 10)
  {
    rv = false;
    reason = "Too many dropped packets";
    level = 2;
    goto end;
  }

  in_lockout_ = bool(this_status->mode_ & MODE_SAFETY_LOCKOUT);
  if (in_lockout_)
  {
    rv = false;
    reason = "Safety Lockout";
    level = 2;
    goto end;
  }

  voltage_estimate_ = prev_status->supply_voltage_ * config_info_.nominal_voltage_scale_ * double(prev_status->programmed_pwm_value_) / 0x4000;

  voltage_error_ = fabs(expected_voltage - voltage_estimate_);
  max_voltage_error_ = max(voltage_error_, max_voltage_error_);
  filtered_voltage_error_ = 0.995 * filtered_voltage_error_ + 0.005 * voltage_error_;
  max_filtered_voltage_error_ = max(filtered_voltage_error_, max_filtered_voltage_error_);

  // Check back-EMF consistency
  if(filtered_voltage_error_ > 6)
  {
    reason = "Problem with the MCB, motor, encoder, or actuator model.";
    //Something is wrong with the encoder, the motor, or the motor board

    //Disable motors
    //rv = false;

    const double epsilon = 0.001;
    //Try to diagnose further
    //motor_velocity == 0 -> encoder failure likely
    if (fabs(state.velocity_) < epsilon)
    {
      reason += " Velocity near zero - check for encoder error.";
      level = 1;
    }
    //measured_current_ ~= 0 -> motor open-circuit likely
    else if (fabs(state.last_measured_current_) < epsilon)
    {
      reason += " Current near zero - check for unconnected motor leads.";
      level = 1;
    }
    //motor_voltage_ ~= 0 -> motor short-circuit likely
    else if (fabs(voltage_estimate_) < epsilon)
    {
      reason += " Voltage near zero - check for short circuit.";
      level = 1;
    }
    //else -> current-sense failure likely
    else
    {
      level = 1;
    }
  }

  //Check current-loop performance
  double last_executed_current;
  last_executed_current =  prev_status->programmed_current_ * config_info_.nominal_current_scale_;
  current_error_ = fabs(state.last_measured_current_ - last_executed_current);

  max_current_error_ = max(current_error_, max_current_error_);

  if ((last_executed_current > 0 ?
         prev_status->programmed_pwm_value_ < 0x2c00 :
         prev_status->programmed_pwm_value_ > -0x2c00))
  {
    filtered_current_error_ = 0.995 * filtered_current_error_ + 0.005 * current_error_;
    max_filtered_current_error_ = max(filtered_current_error_, max_filtered_current_error_);
  }

  if (filtered_current_error_ > 0.2)
  {
    //complain and shut down
    //rv = false;
    reason = "Current loop error too large (MCB failing to hit desired current)";
    level = 1;
  }

end:
  // Only report the first error, until motor is reset.
  if (level && level_ == 0)
  {
    level_ = level;
    reason_ = reason;
  }
  actuator_.state_.halted_ = !rv;
  return rv;
}

bool WG021::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  bool rv = true;
  pr2_hardware_interface::ProjectorState &state = projector_.state_;

  WG021Status *this_status, *prev_status;
  this_status = (WG021Status *)(this_buffer + command_size_);
  prev_status = (WG021Status *)(prev_buffer + command_size_);

  if (!(this_status->mode_ & MODE_ENABLE)) {
    goto end;
  }

  digital_out_.state_.data_ = this_status->digital_out_;

  state.timestamp_us_ = this_status->timestamp_;
  state.falling_timestamp_us_ = this_status->output_stop_timestamp_;
  state.rising_timestamp_us_ = this_status->output_start_timestamp_;

  state.output_ = (this_status->output_status_ & 0x1) == 0x1;
  state.falling_timestamp_valid_ = (this_status->output_status_ & 0x8) == 0x8;
  state.rising_timestamp_valid_ = (this_status->output_status_ & 0x4) == 0x4;

  state.A_ = ((this_status->config0_ >> 4) & 0xf);
  state.B_ = ((this_status->config0_ >> 0) & 0xf);
  state.I_ = ((this_status->config1_ >> 4) & 0xf);
  state.M_ = ((this_status->config1_ >> 0) & 0xf);
  state.L0_ = ((this_status->config2_ >> 4) & 0xf);
  state.L0_ = ((this_status->config2_ >> 0) & 0xf);
  state.pulse_replicator_ = (this_status->general_config_ & 0x1) == 0x1;

  state.last_executed_current_ = this_status->programmed_current_ * config_info_.nominal_current_scale_;
  state.last_measured_current_ = this_status->measured_current_ * config_info_.nominal_current_scale_;

  in_lockout_ = bool(this_status->mode_ & MODE_SAFETY_LOCKOUT);
  if (in_lockout_)
  {
    reason_ = "Safety Lockout";
    level_ = 2;
    rv = false;
    goto end;
  }

end:
  return rv;
}


void WG0X::collectDiagnostics(EthercatCom *com)
{
  //Collect safety disable information through mailbox  
  bool success = true;

  // Have parent collect diagnositcs
  EthercatDevice::collectDiagnostics(com);

  WG0XSafetyDisableStatus s;
  if (readMailbox(com, s.BASE_ADDR, &s, sizeof(s)) != 0) {
    success = false;
  }
    
  WG0XSafetyDisableCounters c;
  if (readMailbox(com, c.BASE_ADDR, &c, sizeof(c)) != 0) {
    success = false;
  }
 
  if (!lockWG0XDiagnostics()) {
    wg0x_collect_diagnostics_.valid_ = false;   // change this even if we did't get the lock
    return;
  }


  wg0x_collect_diagnostics_.valid_ = success;   
  if (success) {
    wg0x_collect_diagnostics_.update(s,c);
  }

  unlockWG0XDiagnostics();
}


int WG0X::sendSpiCommand(EthercatCom *com, WG0XSpiEepromCmd const * cmd)
{
  // Send command
  if (writeMailbox(com, WG0XSpiEepromCmd::SPI_COMMAND_ADDR, cmd, sizeof(*cmd)))
  {
    fprintf(stderr, "ERROR WRITING EEPROM COMMAND\n");
    return -1;
  }

  for (int tries = 0; tries < 10; ++tries)
  {
    WG0XSpiEepromCmd stat;
    if (readMailbox(com, WG0XSpiEepromCmd::SPI_COMMAND_ADDR, &stat, sizeof(stat)))
    {
      fprintf(stderr, "ERROR READING EEPROM BUSY STATUS\n");
      return -1;
    }

    if (stat.operation_ != cmd->operation_)
    {
      fprintf(stderr, "READBACK OF OPERATION INVALID : got 0x%X, expected 0x%X\n", stat.operation_, cmd->operation_);
      return -1;
    }

    // Keep looping while SPI command is running
    if (!stat.busy_)
    {
      return 0;
    }

    fprintf(stderr, "eeprom busy reading again, waiting...\n");
    usleep(100);
  }

  fprintf(stderr, "ERROR : EEPROM READING BUSY AFTER 10 TRIES\n");
  return -1;
}

int WG0X::readEeprom(EthercatCom *com)
{
  assert(sizeof(actuator_info_) == 264);
  WG0XSpiEepromCmd cmd;
  cmd.build_read(ACTUATOR_INFO_PAGE);
  if (sendSpiCommand(com, &cmd)) {
    fprintf(stderr, "ERROR SENDING SPI EEPROM READ COMMAND\n");
    return -1;
  }
  // Read buffered data in multiple chunks
  if (readMailbox(com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, &actuator_info_, sizeof(actuator_info_))) {
    fprintf(stderr, "ERROR READING BUFFERED EEPROM PAGE DATA\n");
    return -1;
  }

  return 0;

}

void WG0X::program(WG0XActuatorInfo *info)
{
  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());

  writeMailbox(&com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, info, sizeof(WG0XActuatorInfo));
  WG0XSpiEepromCmd cmd;
  cmd.build_write(ACTUATOR_INFO_PAGE);
  if (sendSpiCommand(&com, &cmd)) {
    fprintf(stderr, "ERROR SENDING SPI EEPROM WRITE COMMAND\n");
  }

  char data[2];
  memset(data, 0, sizeof(data));
  data[0] = 0xD7;

  if (writeMailbox(&com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, data, sizeof(data))) {
    fprintf(stderr, "ERROR WRITING EEPROM COMMAND BUFFER\n");
  }


  { // Start arbitrary command
    WG0XSpiEepromCmd cmd;
    cmd.build_arbitrary(sizeof(data));
    if (sendSpiCommand(&com, &cmd)) {
      fprintf(stderr, "reading eeprom status failed");
    }
  }


  if (readMailbox(&com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, data, sizeof(data))) {
    fprintf(stderr, "ERROR READING EEPROM COMMAND BUFFER\n");
  }
}

/*!
 * \brief  Find differece between two timespec values
 *
 * \param current   current time 
 * \param current   start time 
 * \return          returns time difference (current-start) in milliseconds
 */
int timediff_ms(const timespec &current, const timespec &start)
{
  int timediff_ms = (current.tv_sec-start.tv_sec)*1000 // 1000 ms in a sec
    + (current.tv_nsec-start.tv_nsec)/1000000; // 1000000 ns in a ms
  return timediff_ms;
}


/*!
 * \brief  error checking wrapper around clock_gettime
 *
 * \param current   current time 
 * \param current   start time 
 * \return          returns 0 for success, non-zero for failure
 */
int safe_clock_gettime(clockid_t clk_id, timespec *time)
{
  int result = clock_gettime(clk_id, time);
  if (result != 0) {
    int error = errno;
    fprintf(stderr, "safe_clock_gettime : %s\n", strerror(error));
    return result;
  }  
  return result;
}


/*!
 * \brief  safe version of usleep.
 *
 * Uses nanosleep internally.  Will restart sleep after begin woken by signal.
 *
 * \param usec   number of microseconds to sleep for.  Must be < 1000000.
 */
void safe_usleep(uint32_t usec) 
{
  assert(usec<1000000);
  if (usec>1000000)
    usec=1000000;
  struct timespec req, rem;
  req.tv_sec = 0;
  req.tv_nsec = usec*1000;
  while (nanosleep(&req, &rem)!=0) { 
    int error = errno;
    fprintf(stderr,"%s : Error : %s\n", __func__, strerror(error));    
    if (error != EINTR) {
      break;
    }
    req = rem;
  }
  return;
}


unsigned SyncMan::baseAddress(unsigned num) 
{
  assert(num < 8);
  return BASE_ADDR + 8 * num;
}  
  

/*!
 * \brief  Read data from Sync Manager
 *
 * \param com       used to perform communication with device
 * \param sh        slave to read data from
 * \param addrMode  addressing mode used to read data (FIXED/POSITIONAL)
 * \param num       syncman number to read 0-7
 * \return          returns true for success, false for failure 
 */
bool SyncMan::readData(EthercatCom *com, EtherCAT_SlaveHandler *sh, EthercatDevice::AddrMode addrMode, unsigned num)
{
  return ( EthercatDevice::readData(com, sh, baseAddress(num), this, sizeof(*this), addrMode) == 0);
}


unsigned SyncManActivate::baseAddress(unsigned num)
{
  assert(num < 8);
  return BASE_ADDR + 8 * num;
}

/*!
 * \brief  Write data to Sync Manager Activation register
 *
 * \param com       used to perform communication with device
 * \param sh        slave to read data from
 * \param addrMode  addressing mode used to read data (FIXED/POSITIONAL)
 * \param num       syncman number to read 0-7
 * \return          returns true for success, false for failure 
 */
bool SyncManActivate::writeData(EthercatCom *com, EtherCAT_SlaveHandler *sh, EthercatDevice::AddrMode addrMode, unsigned num) const
{
  return ( EthercatDevice::writeData(com, sh, baseAddress(num), this, sizeof(*this), addrMode) == 0);
}


void updateIndexAndWkc(EC_Telegram *tg, EC_Logic *logic) 
{
  tg->set_idx(logic->get_idx());
  tg->set_wkc(logic->get_wkc());
}


bool WG0X::verifyDeviceStateForMailboxOperation()
{
  // Make sure slave is in correct state to do use mailbox
  EC_State state = sh_->get_state();
  if ((state != EC_SAFEOP_STATE) && (state != EC_OP_STATE)) {
    fprintf(stderr, "%s : " ERROR_HDR 
            "cannot do mailbox read in current device state = %d\n", __func__, state);
    return false;
  }
  return true;
}


/*!
 * \brief  Runs diagnostic on read and write mailboxes.
 *
 * Collects and data from mailbox control registers.
 *
 * \todo            not implemented yet
 * \param com       used to perform communication with device
 * \return          returns true for success, false for failure 
 */
void WG0X::diagnoseMailboxError(EthercatCom *com)
{
  
}

/*!
 * \brief  Clears read mailbox by reading first and last byte.
 *
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \return          returns true for success, false for failure 
 */
bool WG0X::clearReadMailbox(EthercatCom *com)
{
  if (!verifyDeviceStateForMailboxOperation()){
    return false;
  }

  EC_Logic *logic = EC_Logic::instance();    
  EC_UINT station_addr = sh_->get_station_address();  
  
  // Create Ethernet packet with two EtherCAT telegrams inside of it : 
  //  - One telegram to read first byte of mailbox
  //  - One telegram to read last byte of mailbox
  unsigned char unused[1];
  NPRD_Telegram read_start(
            logic->get_idx(),
            station_addr,
            MBX_STATUS_PHY_ADDR,
            logic->get_wkc(),
            sizeof(unused),
            unused);
  NPRD_Telegram read_end(  
            logic->get_idx(),
            station_addr,
            MBX_STATUS_PHY_ADDR+MBX_STATUS_SIZE-1,
            logic->get_wkc(),
            sizeof(unused),
            unused);
  read_start.attach(&read_end);
  EC_Ethernet_Frame frame(&read_start);


  // Retry sending packet multiple times 
  bool success=false;
  static const unsigned MAX_DROPS = 15;
  for (unsigned tries=0; tries<MAX_DROPS; ++tries) {
    success = com->txandrx_once(&frame);
    if (success) {
      break;
    }
    updateIndexAndWkc(&read_start, logic);
    updateIndexAndWkc(&read_end  , logic);
  }

  if (!success) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " too much packet loss\n", __func__);   
    safe_usleep(100);
    return false;
  }
  
  // Check result for consistancy
  if (read_start.get_wkc() != read_end.get_wkc()) {
    fprintf(stderr, "%s : " ERROR_HDR 
            "read mbx working counters are inconsistant, %d, %d\n",
            __func__, read_start.get_wkc(), read_end.get_wkc());
    return false;
  }
  if (read_start.get_wkc() > 1) {
    fprintf(stderr, "%s : " ERROR_HDR 
            "more than one device (%d) responded \n", __func__, read_start.get_wkc());
    return false;
  }
  if (read_start.get_wkc() == 1)  {
    fprintf(stderr, "%s : " WARN_MODE "WARN" STD_MODE 
            " read mbx contained garbage data\n", __func__);
    // Not an error, just warning
  } 
  
  return true;  
}



/*!
 * \brief  Waits until read mailbox is full or timeout.
 *
 * Wait times out after 100msec.
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \return          returns true for success, false for failure or timeout
 */
bool WG0X::waitForReadMailboxReady(EthercatCom *com)
{
  // Wait upto 100ms for device to toggle ack
  static const int MAX_WAIT_TIME_MS = 100;
  int timediff;
  unsigned good_results=0;


  struct timespec start_time, current_time;
  if (safe_clock_gettime(CLOCK_MONOTONIC, &start_time)!=0) {
    return false;
  }
  
  do {      
    // Check if mailbox is full by looking at bit 3 of SyncMan status register.
    uint8_t SyncManStatus;
    const unsigned SyncManAddr = 0x805+(MBX_STATUS_SYNCMAN_NUM*8);
    if (readData(com, SyncManAddr, &SyncManStatus, sizeof(SyncManStatus), FIXED_ADDR) == 0) {
      ++good_results;
      const uint8_t MailboxStatusMask = (1<<3);
      if (SyncManStatus & MailboxStatusMask) {
        return true;
      }
    }      
    if (safe_clock_gettime(CLOCK_MONOTONIC, &current_time)!=0) {
      return false;
      }
    timediff = timediff_ms(current_time, start_time);
    safe_usleep(100);
  } while (timediff < MAX_WAIT_TIME_MS);
  
  if (good_results == 0) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " error reading from device\n", __func__);          
  } else {
    fprintf(stderr, "%s : " ERROR_HDR 
            " error read mbx not full after %d ms\n", __func__, timediff);      
  }

  return false;
}


/*!
 * \brief  Waits until write mailbox is empty or timeout.
 *
 * Wait times out after 100msec.
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \return          returns true for success, false for failure or timeout
 */
bool WG0X::waitForWriteMailboxReady(EthercatCom *com)
{
  // Wait upto 100ms for device to toggle ack
  static const int MAX_WAIT_TIME_MS = 100;
  int timediff;
  unsigned good_results=0;


  struct timespec start_time, current_time;
  if (safe_clock_gettime(CLOCK_MONOTONIC, &start_time)!=0) {
    return false;
  }
  
  do {      
    // Check if mailbox is full by looking at bit 3 of SyncMan status register.
    uint8_t SyncManStatus;
    const unsigned SyncManAddr = 0x805+(MBX_COMMAND_SYNCMAN_NUM*8);
    if (readData(com, SyncManAddr, &SyncManStatus, sizeof(SyncManStatus), FIXED_ADDR) == 0) {
      ++good_results;
      const uint8_t MailboxStatusMask = (1<<3);
      if ( !(SyncManStatus & MailboxStatusMask) ) {
        return true;
      }
    }      
    if (safe_clock_gettime(CLOCK_MONOTONIC, &current_time)!=0) {
      return false;
    }
    timediff = timediff_ms(current_time, start_time);
    safe_usleep(100);
  } while (timediff < MAX_WAIT_TIME_MS);
  
  if (good_results == 0) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " error reading from device\n", __func__);          
  } else {
    fprintf(stderr, "%s : " ERROR_HDR 
            " error write mbx not empty after %d ms\n", __func__, timediff);      
  }

  return false;
}



/*!
 * \brief  Writes data to mailbox.
 *
 * Will try to conserve bandwidth by only length bytes of data and last byte of mailbox.
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \param data      pointer to buffer where read data is stored.
 * \param length    amount of data to read from mailbox
 * \return          returns true for success, false for failure
 */
bool WG0X::writeMailboxInternal(EthercatCom *com, void const *data, unsigned length)
{
  if (length > MBX_COMMAND_SIZE) {
    assert(length <= MBX_COMMAND_SIZE);
    return false;
  }

  // Make sure slave is in correct state to use mailbox
  if (!verifyDeviceStateForMailboxOperation()){
    return false;
  }

  EC_Logic *logic = EC_Logic::instance();    
  EC_UINT station_addr = sh_->get_station_address();
  

  // If there enough savings, split mailbox write up into 2 parts : 
  //  1. Write of actual data to begining of mbx buffer
  //  2. Write of last mbx buffer byte, to complete write
  static const unsigned TELEGRAM_OVERHEAD = 50;
  bool split_write = (length+TELEGRAM_OVERHEAD) < MBX_COMMAND_SIZE;
    
  unsigned write_length = MBX_COMMAND_SIZE;
  if (split_write) {
    write_length = length;
  }

  // Possible do multiple things at once...
  //  1. Clear read mailbox by reading both first and last mailbox bytes
  //  2. Write data into write mailbox
  {
    // Build frame with 2-NPRD + 2 NPWR
    unsigned char unused[1];
    NPWR_Telegram write_start(
                              logic->get_idx(),
                              station_addr,
                              MBX_COMMAND_PHY_ADDR,
                              logic->get_wkc(),
                              write_length,
                              (const unsigned char*) data);
    NPWR_Telegram write_end(
                            logic->get_idx(),
                            station_addr,
                            MBX_COMMAND_PHY_ADDR+MBX_COMMAND_SIZE-1,
                            logic->get_wkc(),
                            sizeof(unused),
                            unused);
      
    if (split_write) {
      write_start.attach(&write_end);
    }      

    EC_Ethernet_Frame frame(&write_start);
      
    // Try multiple times, but remember number of of successful sends
    unsigned sends=0;      
    bool success=false;
    for (unsigned tries=0; (tries<10) && !success; ++tries) {
      success = com->txandrx_once(&frame);
      if (!success) {
        updateIndexAndWkc(&write_start, logic);
        updateIndexAndWkc(&write_end, logic);
      }
      ++sends; //EtherCAT_com d/n support split TX and RX class, assume tx part of txandrx always succeeds
      /* 
      int handle = com->tx(&frame);
      if (handle > 0) {
        ++sends;
        success = com->rx(&frame, handle);
      }
      if (!success) {
        updateIndexAndWkc(&write_start, logic);
        updateIndexAndWkc(&write_end, logic);
      }
      */
    }
    if (!success) {
      fprintf(stderr, "%s : " ERROR_HDR 
              " too much packet loss\n", __func__);   
      safe_usleep(100);
      return false;
    }
      
    if (split_write && (write_start.get_wkc() != write_end.get_wkc())) {
      fprintf(stderr, "%s : " ERROR_HDR 
              " write mbx working counters are inconsistant\n", __func__);
      return false;
    }

    if (write_start.get_wkc() != 1)
    {
      // Write to cmd mbx was refused 
      if (sends<=1) {
        // Packet was only sent once, there must be a problem with slave device
        fprintf(stderr, "%s : " ERROR_HDR 
                " initial mailbox write refused\n", __func__);
        safe_usleep(100);
        return false;
      } else {
        // Packet was sent multiple times because a packet drop occured  
        // If packet drop occured on return path from device, a refusal is acceptable
        fprintf(stderr, "%s : " WARN_HDR 
                " repeated mailbox write refused\n", __func__);
      }
    }     
  }

  return true;
}

bool WG0X::readMailboxRepeatRequest(EthercatCom *com)
{
  bool success = _readMailboxRepeatRequest(com);
  ++mailbox_diagnostics_.retries_;
  if (!success) {
    ++mailbox_diagnostics_.retry_errors_;
  }
  return success;
}

bool WG0X::_readMailboxRepeatRequest(EthercatCom *com)
{
  // Toggle repeat request flag, wait for ack from device
  // Returns true if ack is received, false for failure
  SyncMan sm;
  if (!sm.readData(com, sh_, FIXED_ADDR, MBX_STATUS_SYNCMAN_NUM)) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " could not read status mailbox syncman (1)\n", __func__);
    return false;
  }
  
  // If device can handle repeat requests, then request and ack bit should already match
  if (sm.activate.repeat_request != sm.pdi_control.repeat_ack) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " syncman repeat request and ack do not match\n", __func__);
    return false;
  }

  // Write toggled repeat request,,, wait for ack.
  SyncManActivate orig_activate(sm.activate);
  sm.activate.repeat_request = ~orig_activate.repeat_request;
  if (!sm.activate.writeData(com, sh_, FIXED_ADDR, MBX_STATUS_SYNCMAN_NUM)) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " could not write syncman repeat request\n", __func__);
    //ec_mark(sh->getEM(), "could not write syncman repeat request", 1);
    return false;
  }
  
  // Wait upto 100ms for device to toggle ack
  static const int MAX_WAIT_TIME_MS = 100;
  int timediff;

  struct timespec start_time, current_time;
  if (safe_clock_gettime(CLOCK_MONOTONIC, &start_time)!=0) {
    return false;
  }
  
  do {
    if (!sm.readData(com, sh_, FIXED_ADDR, MBX_STATUS_SYNCMAN_NUM)) {
      fprintf(stderr, "%s : " ERROR_HDR 
              " could not read status mailbox syncman (2)\n", __func__);
      return false;
    }

    if (sm.activate.repeat_request == sm.pdi_control.repeat_ack) {
      // Device responded, to some checks to make sure it seems to be telling the truth
      if (sm.status.mailbox_status != 1) {
        fprintf(stderr, "%s : " ERROR_HDR 
                " got repeat response, but read mailbox is still empty\n", __func__);
        //sm.print(WG0X_MBX_Status_Syncman_Num, std::cerr);
        return false;
      }
      return true;
    }
    
    if ( (sm.activate.repeat_request) == (orig_activate.repeat_request) ) {          
      fprintf(stderr, "%s : " ERROR_HDR 
              " syncman repeat request was changed while waiting for response\n", __func__);
      //sm.activate.print();
      //orig_activate.print();
      return false;
    }

    if (safe_clock_gettime(CLOCK_MONOTONIC, &current_time)!=0) {
      return false;
    }
    
    timediff = timediff_ms(current_time, start_time);
    safe_usleep(100);        
  } while (timediff < MAX_WAIT_TIME_MS);
    
  fprintf(stderr, "%s : " ERROR_HDR 
          " error repeat request not acknowledged after %d ms\n", __func__, timediff);    
  return false;
}



/*!
 * \brief  Reads data from read mailbox.
 *
 * Will try to conserve bandwidth by reading length bytes of data and last byte of mailbox.
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \param data      pointer to buffer where read data is stored.
 * \param length    amount of data to read from mailbox
 * \return          returns true for success, false for failure
 */
bool WG0X::readMailboxInternal(EthercatCom *com, void *data, unsigned length)
{
  static const unsigned MAX_TRIES = 10;
  static const unsigned MAX_DROPPED = 10;
    
  if (length > MBX_STATUS_SIZE) {
    assert(length <= MBX_STATUS_SIZE);
    return false;
  }

  // Make sure slave is in correct state to use mailbox
  if (!verifyDeviceStateForMailboxOperation()){
    return false;
  }
    
  EC_Logic *logic = EC_Logic::instance();    
  EC_UINT station_addr = sh_->get_station_address();


  // If read is small enough :
  //  1. read just length bytes in one telegram
  //  2. then read last byte to empty mailbox
  static const unsigned TELEGRAM_OVERHEAD = 50;
  bool split_read = (length+TELEGRAM_OVERHEAD) < MBX_STATUS_SIZE;
    
  unsigned read_length = MBX_STATUS_SIZE;      
  if (split_read) {
    read_length = length;
 }

  unsigned char unused[1];
  NPRD_Telegram read_start(
                           logic->get_idx(),
                           station_addr,
                           MBX_STATUS_PHY_ADDR,
                           logic->get_wkc(),
                           read_length,
                           (unsigned char*) data);
  NPRD_Telegram read_end(  
                         logic->get_idx(),
                         station_addr,
                         MBX_STATUS_PHY_ADDR+MBX_STATUS_SIZE-1,
                         logic->get_wkc(),
                         sizeof(unused),
                         unused);      

  if (split_read) {
    read_start.attach(&read_end);
  }
    
  EC_Ethernet_Frame frame(&read_start);

  unsigned tries = 0;    
  unsigned total_dropped =0;
  for (tries=0; tries<MAX_TRIES; ++tries) {      

    // Send read - keep track of how many packets were dropped (for later)
    unsigned dropped=0;
    for (dropped=0; dropped<MAX_DROPPED; ++dropped) {
      if (com->txandrx_once(&frame)) {
        break;
      }
      ++total_dropped;
      updateIndexAndWkc(&read_start   , logic);
      updateIndexAndWkc(&read_end     , logic);
    }
      
    if (dropped>=MAX_DROPPED) {
      fprintf(stderr, "%s : " ERROR_HDR 
              " too many dropped packets : %d\n", __func__, dropped);
    }
      
    if (split_read && (read_start.get_wkc() != read_end.get_wkc())) {
      fprintf(stderr, "%s : " ERROR_HDR 
              "read mbx working counters are inconsistant\n", __func__);
      return false;
    }
      
    if (read_start.get_wkc() == 0) {
      if (dropped == 0) {
        fprintf(stderr, "%s : " ERROR_HDR 
                " inconsistancy : got wkc=%d with no dropped packets\n", 
                __func__, read_start.get_wkc()); 
        fprintf(stderr, "total dropped = %d\n", total_dropped);
        return false;
      } else {
        // Packet was dropped after doing read from device,,,
        // Ask device to repost data, so it can be read again.
        fprintf(stderr, "%s : " WARN_HDR 
                " asking for read repeat after dropping %d packets\n", __func__, dropped);
        if (!readMailboxRepeatRequest(com)) {
          return false;
        }
        continue;
      }
    } else if (read_start.get_wkc() == 1) {
      // Successfull read of status data
      break;
    } else {
      fprintf(stderr, "%s : " ERROR_HDR 
              " invalid wkc for read : %d\n", __func__, read_start.get_wkc());   
      diagnoseMailboxError(com);
      return false;
    }
  }

  if (tries >= MAX_TRIES) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " could not get responce from device after %d retries, %d total dropped packets\n",
            __func__, tries, total_dropped);
    diagnoseMailboxError(com);
    return false;
  }        

  return true;
}


/*!
 * \brief  Read data from WG0X local bus using mailbox communication.
 *
 * Internally a localbus read is done in two parts.
 * First, a mailbox write of a command header that include local bus address and length.
 * Second, a mailbox read of the result.
 *
 * \param com       used to perform communication with device
 * \param address   WG0X (FPGA) local bus address to read from
 * \param data      pointer to buffer where read data can be stored, must be at least length in size
 * \param length    amount of data to read, limited at 511 bytes.
 * \return          returns zero for success, non-zero for failure 
 */
int WG0X::readMailbox(EthercatCom *com, unsigned address, void *data, unsigned length)
{
  if (!lockMailbox())
    return -1;

  int result = readMailbox_(com, address, data, length);
  if (result != 0) {
    ++mailbox_diagnostics_.read_errors_;
  }
  
  unlockMailbox();
  return result;
}

/*!
 * \brief  Internal function.  
 *
 * Aguments are the same as readMailbox() except that this assumes the mailbox lock is held.
 */ 
int WG0X::readMailbox_(EthercatCom *com, unsigned address, void *data, unsigned length)
{
  // Make sure slave is in correct state to use mailbox
  if (!verifyDeviceStateForMailboxOperation()){
    return false;
  }

  //  1. Clear read (status) mailbox by reading it first
  if (!clearReadMailbox(com)) 
  {
    fprintf(stderr, "%s : " ERROR_HDR 
            " clearing read mbx\n", __func__);
    return -1;
  }

  //  2. Put a (read) request into command mailbox
  {
    WG0XMbxCmd cmd;      
    if (!cmd.build(address, length, LOCAL_BUS_READ, sh_->get_mbx_counter(), data)) 
    {
      fprintf(stderr, "%s : " ERROR_HDR 
              " builing mbx header\n", __func__);
      return -1;
    }
    
    if (!writeMailboxInternal(com, &cmd.hdr_, sizeof(cmd.hdr_))) 
    {
      fprintf(stderr, "%s : " ERROR_HDR " write of cmd failed\n", __func__);
      return -1;
    }
  }
  
  // Wait for result (in read mailbox) to become ready
  if (!waitForReadMailboxReady(com)) 
  {
    fprintf(stderr, "%s : " ERROR_HDR 
            "waiting for read mailbox\n", __func__);
    return -1;
  }

  // Read result back from mailbox.
  // It could take the FPGA some time to respond to a request.  
  // Since the read mailbox is initiall cleared, any read to the mailbox
  // should be refused (WKC==0) until WG0x FPGA has written it result into it.	   
  // NOTE: For this to work the mailbox syncmanagers must be set up.
  // TODO 1: Packets may get lost on return route to device.
  //   In this case, the device will keep responding to the repeated packets with WKC=0.
  //   To work correctly, the repeat request bit needs to be toggled.
  // TODO 2: Need a better method to determine if data read from status mailbox.
  //   is the right data, or just junk left over from last time.
  { 
    WG0XMbxCmd stat;
    // Read data + 1byte checksum from mailbox
    if (!readMailboxInternal(com, &stat, length+1)) 
    {
      fprintf(stderr, "%s : " ERROR_HDR " read failed\n", __func__);
      return -1;
    }
    
    if (computeChecksum(&stat, length+1) != 0) 
    {
      fprintf(stderr, "%s : " ERROR_HDR 
              "checksum error reading mailbox data\n", __func__);
      fprintf(stderr, "length = %d\n", length);
      return -1;
    }
    memcpy(data, &stat, length);
  }

  return 0;


}

bool WG0X::lockMailbox() 
{
  int error = pthread_mutex_lock(&mailbox_lock_);
  if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " getting mbx lock\n", __func__);
    ++mailbox_diagnostics_.lock_errors_;
    return false;
  }
  return true;
}

void WG0X::unlockMailbox() 
{
  int error = pthread_mutex_unlock(&mailbox_lock_);
  if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " freeing mbx lock\n", __func__);
    ++mailbox_diagnostics_.lock_errors_;
  }
}

bool WG0X::lockWG0XDiagnostics() 
{
  int error = pthread_mutex_lock(&wg0x_diagnostics_lock_);
  if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " getting diagnostics lock\n", __func__);
    // update error counters even if we didn't get lock
    ++wg0x_collect_diagnostics_.lock_errors_;
    return false;
  }
  return true;
}

bool WG0X::tryLockWG0XDiagnostics() 
{
  int error = pthread_mutex_trylock(&wg0x_diagnostics_lock_);
  if (error == EBUSY) {
    return false;
  }
  else if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " getting diagnostics lock\n", __func__);
    // update error counters even if we didn't get lock
    ++wg0x_collect_diagnostics_.lock_errors_;
    return false;
  }
  return true;
}

void WG0X::unlockWG0XDiagnostics() 
{
  int error = pthread_mutex_unlock(&wg0x_diagnostics_lock_);
  if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " freeing diagnostics lock\n", __func__);
    ++wg0x_collect_diagnostics_.lock_errors_;
  }
}


/*!
 * \brief  Write data to WG0X local bus using mailbox communication.
 *
 * First, this puts a command header that include local bus address and length in write mailbox.
 * Second it waits until device actually empties write mailbox.
 *
 * \param com       used to perform communication with device
 * \param address   WG0X (FPGA) local bus address to write data to
 * \param data      pointer to buffer where write data is stored, must be at least length in size
 * \param length    amount of data to write, limited at 507 bytes
 * \return          returns zero for success, non-zero for failure 
 */
int WG0X::writeMailbox(EthercatCom *com, unsigned address, void const *data, unsigned length)
{
  if (!lockMailbox())
    return -1;

  int result = writeMailbox_(com, address, data, length);
  if (result != 0) {
    ++mailbox_diagnostics_.write_errors_;
  }

  unlockMailbox();

  return result;
}

/*!
 * \brief  Internal function.  
 *
 * Aguments are the same as writeMailbox() except that this assumes the mailbox lock is held.
 */
int WG0X::writeMailbox_(EthercatCom *com, unsigned address, void const *data, unsigned length)
{
  // Make sure slave is in correct state to use mailbox
  if (!verifyDeviceStateForMailboxOperation()){
    return -1;
  }
    
  // Build message and put it into write mailbox
  {		
    WG0XMbxCmd cmd;
    if (!cmd.build(address, length, LOCAL_BUS_WRITE, sh_->get_mbx_counter(), data)) {
      fprintf(stderr, "%s : " ERROR_HDR " builing mbx header\n", __func__);
      return -1;
    }      
    
    unsigned write_length = sizeof(cmd.hdr_)+length+sizeof(cmd.checksum_);
    if (!writeMailboxInternal(com, &cmd, write_length)) {
      fprintf(stderr, "%s : " ERROR_HDR " write failed\n", __func__);
      diagnoseMailboxError(com);
      return -1;
    }
  }
  
  // TODO: Change slave firmware so that we can verify that localbus write was truly executed
  //  Checking that device emptied write mailbox will have to suffice for now.
  if (!waitForWriteMailboxReady(com)) {
    fprintf(stderr, "%s : " ERROR_HDR 
            "write mailbox\n", __func__);
  }
    
  return 0;
}


#define CHECK_SAFETY_BIT(bit) \
  do { if (status & SAFETY_##bit) { \
    str += prefix + #bit; \
    prefix = ", "; \
  } } while(0)

string WG0X::safetyDisableString(uint8_t status)
{
  string str, prefix;

  if (status & SAFETY_DISABLED)
  {
    CHECK_SAFETY_BIT(DISABLED);
    CHECK_SAFETY_BIT(UNDERVOLTAGE);
    CHECK_SAFETY_BIT(OVER_CURRENT);
    CHECK_SAFETY_BIT(BOARD_OVER_TEMP);
    CHECK_SAFETY_BIT(HBRIDGE_OVER_TEMP);
    CHECK_SAFETY_BIT(OPERATIONAL);
    CHECK_SAFETY_BIT(WATCHDOG);
  }
  else
    str = "ENABLED";

  return str;
}

void WG0X::publishMailboxDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
{
  if (lockMailbox()) { 
    mailbox_publish_diagnostics_ = mailbox_diagnostics_;
    unlockMailbox();
  }

  MbxDiagnostics const &m(mailbox_publish_diagnostics_);
  d.addf("Mailbox Write Errors", "%d", m.write_errors_);
  d.addf("Mailbox Read Errors", "%d",  m.read_errors_);
  d.addf("Mailbox Retries", "%d",      m.retries_);
  d.addf("Mailbox Retry Errors", "%d", m.retry_errors_);
}

void WG0X::publishGeneralDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
{ 
  // If possible, copy new diagnositics from collection thread, into diagnostics thread
  if (tryLockWG0XDiagnostics()) { 
    wg0x_publish_diagnostics_ = wg0x_collect_diagnostics_;
    unlockWG0XDiagnostics(); 
  }

  if (!wg0x_publish_diagnostics_.valid_) 
  {
    d.mergeSummaryf(d.WARN, "Could not collect WG0X diagnostics");
  }

  WG0XDiagnostics const &p(wg0x_publish_diagnostics_);
  WG0XSafetyDisableStatus const &s(p.safety_disable_status_);
  d.addf("Safety Disable Status", "%s (%02x)", safetyDisableString(s.safety_disable_status_).c_str(), s.safety_disable_status_);
  d.addf("Safety Disable Status Hold", "%s (%02x)", safetyDisableString(s.safety_disable_status_hold_).c_str(), s.safety_disable_status_hold_);
  d.addf("Safety Disable Count", "%d", p.safety_disable_total_);
  d.addf("Undervoltage Count", "%d", p.undervoltage_total_);
  d.addf("Over Current Count", "%d", p.over_current_total_);
  d.addf("Board Over Temp Count", "%d", p.board_over_temp_total_);
  d.addf("Bridge Over Temp Count", "%d", p.bridge_over_temp_total_);
  d.addf("Operate Disable Count", "%d", p.operate_disable_total_);
  d.addf("Watchdog Disable Count", "%d", p.watchdog_disable_total_);
}

void WG0X::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  WG0XStatus *status = (WG0XStatus *)(buffer + command_size_);

  stringstream str;
  str << "EtherCAT Device (" << actuator_info_.name_ << ")";
  d.name = str.str();
  char serial[32];
  snprintf(serial, sizeof(serial), "%d-%05d-%05d", config_info_.product_id_ / 100000 , config_info_.product_id_ % 100000, config_info_.device_serial_number_);
  d.hardware_id = serial;


  d.summary(level_, reason_);

  d.clear();
  d.add("Configuration", config_info_.configuration_status_ ? "good" : "error loading configuration");
  d.add("Name", actuator_info_.name_);
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product code",
        "WG0%d (%d) Firmware Revision %d.%02d, PCB Revision %c.%02d",
        sh_->get_product_code() == WG05::PRODUCT_CODE ? 5 : 6,
        sh_->get_product_code(), fw_major_, fw_minor_,
        'A' + board_major_, board_minor_);

  d.add("Robot", actuator_info_.robot_name_);
  d.addf("Motor", "%s %s", actuator_info_.motor_make_, actuator_info_.motor_model_);
  d.add("Serial Number", serial);
  d.addf("Nominal Current Scale", "%f",  config_info_.nominal_current_scale_);
  d.addf("Nominal Voltage Scale",  "%f", config_info_.nominal_voltage_scale_);
  d.addf("HW Max Current", "%f", config_info_.absolute_current_limit_ * config_info_.nominal_current_scale_);

  d.addf("SW Max Current", "%f", actuator_info_.max_current_);
  d.addf("Speed Constant", "%f", actuator_info_.speed_constant_);
  d.addf("Resistance", "%f", actuator_info_.resistance_);
  d.addf("Motor Torque Constant", "%f", actuator_info_.motor_torque_constant_);
  d.addf("Pulses Per Revolution", "%d", actuator_info_.pulses_per_revolution_);
  d.addf("Encoder Reduction", "%f\n", actuator_info_.encoder_reduction_);

  publishGeneralDiagnostics(d);
  publishMailboxDiagnostics(d);

  d.addf("Watchdog Limit", "%dms\n", config_info_.watchdog_limit_);

  string mode, prefix;
  if (status->mode_) {
    if (status->mode_ & MODE_ENABLE) {
      mode += prefix + "ENABLE";
      prefix = ", ";
    }
    if (status->mode_ & MODE_CURRENT) {
      mode += prefix + "CURRENT";
      prefix = ", ";
    }
    if (status->mode_ & MODE_UNDERVOLTAGE) {
      mode += prefix + "UNDERVOLTAGE";
      prefix = ", ";
    }
    if (status->mode_ & MODE_SAFETY_RESET) {
      mode += prefix + "SAFETY_RESET";
      prefix = ", ";
    }
    if (status->mode_ & MODE_SAFETY_LOCKOUT) {
      mode += prefix + "SAFETY_LOCKOUT";
      prefix = ", ";
    }
    if (status->mode_ & MODE_RESET) {
      mode += prefix + "RESET";
      prefix = ", ";
    }
  } else {
    mode = "OFF";
  }
  d.add("Mode", mode);
  d.addf("Digital out", "%d", status->digital_out_);
  d.addf("Programmed pwm value", "%d", status->programmed_pwm_value_);
  d.addf("Programmed current", "%f", status->programmed_current_ * config_info_.nominal_current_scale_);
  d.addf("Measured current", "%f", status->measured_current_ * config_info_.nominal_current_scale_);
  d.addf("Timestamp", "%u", status->timestamp_);
  d.addf("Encoder count", "%d", status->encoder_count_);
  d.addf("Encoder index pos", "%d", status->encoder_index_pos_);
  d.addf("Num encoder_errors", "%d", status->num_encoder_errors_);
  d.addf("Encoder status", "%d", status->encoder_status_);
  d.addf("Calibration reading", "%d", status->calibration_reading_);
  d.addf("Last calibration rising edge", "%d", status->last_calibration_rising_edge_);
  d.addf("Last calibration falling edge", "%d", status->last_calibration_falling_edge_);
  d.addf("Board temperature", "%f", 0.0078125 * status->board_temperature_);
  d.addf("Max board temperature", "%f", 0.0078125 * max_board_temperature_);
  d.addf("Bridge temperature", "%f", 0.0078125 * status->bridge_temperature_);
  d.addf("Max bridge temperature", "%f", 0.0078125 * max_bridge_temperature_);
  d.addf("Supply voltage", "%f", status->supply_voltage_ * config_info_.nominal_voltage_scale_);
  d.addf("Motor voltage", "%f", status->motor_voltage_ * config_info_.nominal_voltage_scale_);
  d.addf("Current Loop Kp", "%d", config_info_.current_loop_kp_);
  d.addf("Current Loop Ki", "%d", config_info_.current_loop_ki_);

  d.addf("Motor voltage estimate", "%f", voltage_estimate_);
  d.addf("Packet count", "%d", status->packet_count_);

  d.addf("Voltage Error", "%f", voltage_error_);
  d.addf("Max Voltage Error", "%f", max_voltage_error_);
  d.addf("Filtered Voltage Error", "%f", filtered_voltage_error_);
  d.addf("Max Filtered Voltage Error", "%f", max_filtered_voltage_error_);
  d.addf("Current Error", "%f", current_error_);
  d.addf("Max Current Error", "%f", max_current_error_);
  d.addf("Filtered Current Error", "%f", filtered_current_error_);
  d.addf("Max Filtered Current Error", "%f", max_filtered_current_error_);

  d.addf("Drops", "%d", drops_);
  d.addf("Consecutive Drops", "%d", consecutive_drops_);
  d.addf("Max Consecutive Drops", "%d", max_consecutive_drops_);

  unsigned numPorts = (sh_->get_product_code()==WG06::PRODUCT_CODE) ? 1 : 2; // WG006 has 1 port, WG005 has 2
  EthercatDevice::ethercatDiagnostics(d, numPorts); 
}

void WG021::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  WG021Status *status = (WG021Status *)(buffer + command_size_);

  stringstream str;
  str << "EtherCAT Device (" << actuator_info_.name_ << ")";
  d.name = str.str();
  char serial[32];
  snprintf(serial, sizeof(serial), "%d-%05d-%05d", config_info_.product_id_ / 100000 , config_info_.product_id_ % 100000, config_info_.device_serial_number_);
  d.hardware_id = serial;

  d.summary(level_, reason_);

  d.clear();
  d.add("Configuration", config_info_.configuration_status_ ? "good" : "error loading configuration");
  d.add("Name", actuator_info_.name_);
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product code",
        "WG021 (%d) Firmware Revision %d.%02d, PCB Revision %c.%02d",
        sh_->get_product_code(), fw_major_, fw_minor_,
        'A' + board_major_, board_minor_);

  d.add("Robot", actuator_info_.robot_name_);
  d.add("Serial Number", serial);
  d.addf("Nominal Current Scale", "%f",  config_info_.nominal_current_scale_);
  d.addf("Nominal Voltage Scale",  "%f", config_info_.nominal_voltage_scale_);
  d.addf("HW Max Current", "%f", config_info_.absolute_current_limit_ * config_info_.nominal_current_scale_);

  publishGeneralDiagnostics(d);
  publishMailboxDiagnostics(d);

  string mode, prefix;
  if (status->mode_) {
    if (status->mode_ & MODE_ENABLE) {
      mode += prefix + "ENABLE";
      prefix = ", ";
    }
    if (status->mode_ & MODE_CURRENT) {
      mode += prefix + "CURRENT";
      prefix = ", ";
    }
    if (status->mode_ & MODE_UNDERVOLTAGE) {
      mode += prefix + "UNDERVOLTAGE";
      prefix = ", ";
    }
    if (status->mode_ & MODE_SAFETY_RESET) {
      mode += prefix + "SAFETY_RESET";
      prefix = ", ";
    }
    if (status->mode_ & MODE_SAFETY_LOCKOUT) {
      mode += prefix + "SAFETY_LOCKOUT";
      prefix = ", ";
    }
    if (status->mode_ & MODE_RESET) {
      mode += prefix + "RESET";
      prefix = ", ";
    }
  } else {
    mode = "OFF";
  }
  d.add("Mode", mode);
  d.addf("Digital out", "%d", status->digital_out_);
  d.addf("Programmed current", "%f", status->programmed_current_ * config_info_.nominal_current_scale_);
  d.addf("Measured current", "%f", status->measured_current_ * config_info_.nominal_current_scale_);
  d.addf("Timestamp", "%u", status->timestamp_);
  d.addf("Config 0", "%#02x", status->config0_);
  d.addf("Config 1", "%#02x", status->config1_);
  d.addf("Config 2", "%#02x", status->config2_);
  d.addf("Output Status", "%#02x", status->output_status_);
  d.addf("Output Start Timestamp", "%u", status->output_start_timestamp_);
  d.addf("Output Stop Timestamp", "%u", status->output_stop_timestamp_);
  d.addf("Board temperature", "%f", 0.0078125 * status->board_temperature_);
  d.addf("Max board temperature", "%f", 0.0078125 * max_board_temperature_);
  d.addf("Bridge temperature", "%f", 0.0078125 * status->bridge_temperature_);
  d.addf("Max bridge temperature", "%f", 0.0078125 * max_bridge_temperature_);
  d.addf("Supply voltage", "%f", status->supply_voltage_ * config_info_.nominal_voltage_scale_);
  d.addf("LED voltage", "%f", status->led_voltage_ * config_info_.nominal_voltage_scale_);
  d.addf("Packet count", "%d", status->packet_count_);

  EthercatDevice::ethercatDiagnostics(d, 2); 
}
