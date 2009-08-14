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

bool reg05 = DeviceFactory::Instance().Register(WG05::PRODUCT_CODE, deviceCreator<WG05> );
bool reg06 = DeviceFactory::Instance().Register(WG06::PRODUCT_CODE, deviceCreator<WG06> );

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

void WG0XMbxHdr::build(uint16_t address, uint16_t length, bool write_nread)
{
  address_ = address;
  length_ = length - 1;
  pad_ = 0;
  write_nread_ = write_nread;
  checksum_ = rotateRight8(computeChecksum(this, sizeof(*this) - 1));
}

bool WG0XMbxHdr::verifyChecksum(void) const
{
  return computeChecksum(this, sizeof(*this)) != 0;
}

void WG0XMbxCmd::build(unsigned address, unsigned length, bool write_nread, void const *data)
{
  this->hdr_.build(address, length, write_nread);
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


WG0X::WG0X(EtherCAT_SlaveHandler *sh, int &startAddress) : EthercatDevice(sh, true)
{
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
  in_lockout_ = false;

  fw_major_ = (sh->get_revision() >> 8) & 0xff;
  fw_minor_ = sh->get_revision() & 0xff;
  board_major_ = ((sh->get_revision() >> 24) & 0xff) - 1;
  board_minor_ = (sh->get_revision() >> 16) & 0xff;

  bool isWG06 = sh_->get_product_code() == WG06::PRODUCT_CODE;
  unsigned int base_status = sizeof(WG0XStatus);

  command_size_ = sizeof(WG0XCommand);
  status_size_ = sizeof(WG0XStatus);
  if (isWG06)
  {
    if (fw_major_ >= 1)
      status_size_ = base_status = sizeof(WG06StatusWithAccel);
    status_size_ += sizeof(WG06Pressure);
  }

  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(isWG06 ? 3 : 2);
  (*fmmu)[0] = EC_FMMU(startAddress, // Logical start address
                       command_size_,// Logical length
                       0x00, // Logical StartBit
                       0x07, // Logical EndBit
                       COMMAND_PHY_ADDR, // Physical Start address
                       0x00, // Physical StartBit
                       false, // Read Enable
                       true, // Write Enable
                       true); // Enable

  startAddress += command_size_;

  (*fmmu)[1] = EC_FMMU(startAddress, // Logical start address
                       base_status, // Logical length
                       0x00, // Logical StartBit
                       0x07, // Logical EndBit
                       STATUS_PHY_ADDR, // Physical Start address
                       0x00, // Physical StartBit
                       true, // Read Enable
                       false, // Write Enable
                       true); // Enable

  startAddress += base_status;

  if (isWG06)
  {
    (*fmmu)[2] = EC_FMMU(startAddress, // Logical start address
                         sizeof(WG06Pressure), // Logical length
                         0x00, // Logical StartBit
                         0x07, // Logical EndBit
                         PRESSURE_PHY_ADDR, // Physical Start address
                         0x00, // Physical StartBit
                         true, // Read Enable
                         false, // Write Enable
                         true); // Enable

    startAddress += sizeof(WG06Pressure);
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

int WG06::initialize(Actuator *actuator, bool allow_unprogrammed)
{
  int retval = WG0X::initialize(actuator, allow_unprogrammed);
  
  if (!retval && use_ros_)
  {
    string topic = "pressure";
    if (!actuator->name_.empty())
      topic = topic + "/" + string(actuator->name_);
    pressure_publisher_ = new realtime_tools::RealtimePublisher<pr2_msgs::PressureState>(ros::NodeHandle(), topic, 1);

    if (fw_major_ >= 1)
    {
      topic = "/accelerometer/";
      if (!actuator->name_.empty())
        topic += actuator->name_;
      accel_publisher_ = new realtime_tools::RealtimePublisher<pr2_msgs::AccelerometerState>(ros::NodeHandle(), topic, 1);
    }

  }

  return retval;
}

int WG0X::initialize(Actuator *actuator, bool allow_unprogrammed)
{
  ROS_DEBUG("Device #%02d: WG0%d (%#08x) Firmware Revision %d.%02d, PCB Revision %c.%02d", sh_->get_ring_position(),
         sh_->get_product_code() == WG05::PRODUCT_CODE ? 5 : 6,
         sh_->get_product_code(), fw_major_, fw_minor_,
         'A' + board_major_, board_minor_);

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

  if (readMailbox(sh_, WG0XConfigInfo::CONFIG_INFO_BASE_ADDR, &config_info_, sizeof(config_info_)) != 0)
  {
    ROS_FATAL("Unable to load configuration information");
    ROS_BREAK();
    return -1;
  }
  ROS_DEBUG("            Serial #: %05d", config_info_.device_serial_number_);

  if (readEeprom(sh_) < 0)
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

    actuator->name_ = actuator_info_.name_;
    backemf_constant_ = 1.0 / (actuator_info_.speed_constant_ * 2 * M_PI * 1.0/60);
    ROS_DEBUG("            Name: %s", actuator_info_.name_);
  }
  else if (allow_unprogrammed)
  {
    ROS_WARN("WARNING: Device #%02d is not programmed", sh_->get_ring_position());
    actuator_info_.crc32_ = 0;
  }
  else
  {
    ROS_FATAL("Device #%02d: Invalid CRC32 in actuator_info_", sh_->get_ring_position());
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

void WG0X::convertCommand(ActuatorCommand &command, unsigned char *buffer)
{
  WG0XCommand *c = (WG0XCommand *)buffer;

  memset(c, 0, command_size_);

  c->programmed_current_ = int(command.current_ / config_info_.nominal_current_scale_);
  c->mode_ = command.enable_ ? (MODE_ENABLE | MODE_CURRENT | MODE_SAFETY_RESET) : MODE_OFF;
  c->checksum_ = rotateRight8(computeChecksum(c, command_size_ - 1));
  c->digital_out_ = command.digital_out_;
}

void WG0X::computeCurrent(ActuatorCommand &command)
{
  command.current_ = (command.effort_ / actuator_info_.encoder_reduction_) / actuator_info_.motor_torque_constant_ ;
}

void WG0X::truncateCurrent(ActuatorCommand &command)
{
  command.current_ = max(min(command.current_, actuator_info_.max_current_), -actuator_info_.max_current_);
}
  
void WG06::convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer)
{
  int status_bytes = accel_publisher_ ? sizeof(WG06StatusWithAccel) : sizeof(WG0XStatus);

  WG06Pressure *p = (WG06Pressure *)(current_buffer + command_size_ + status_bytes);

  if (p->timestamp_ != last_pressure_time_)
  {
    if (pressure_publisher_ && pressure_publisher_->trylock())
    {
      pressure_publisher_->msg_.header.stamp = ros::Time::now();
      pressure_publisher_->msg_.set_data0_size(22);
      pressure_publisher_->msg_.set_data1_size(22);
      for (int i = 0; i < 22; ++i ) {
        pressure_publisher_->msg_.data0[i] = ((p->data0_[i] >> 8) & 0xff) | ((p->data0_[i] << 8) & 0xff00);
        pressure_publisher_->msg_.data1[i] = ((p->data1_[i] >> 8) & 0xff) | ((p->data1_[i] << 8) & 0xff00);
      }
      pressure_publisher_->unlockAndPublish();
    }
  }

  if (accel_publisher_ && accel_publisher_->trylock())
  {
    WG06StatusWithAccel *status = (WG06StatusWithAccel *)(current_buffer + command_size_);
    WG06StatusWithAccel *last_status = (WG06StatusWithAccel *)(last_buffer + command_size_);
    int count = min(uint8_t(4), uint8_t(status->accel_count_ - last_status->accel_count_));

    accel_publisher_->msg_.set_x_size(count);
    accel_publisher_->msg_.set_y_size(count);
    accel_publisher_->msg_.set_z_size(count);
    for (int i = 0; i < count; ++i)
    {
      int32_t acc = status->accel_[count - i - 1];
      int range = (acc >> 30) & 3;
      float d = 1 << (8 - range);
      accel_publisher_->msg_.x[i] = ((((acc >>  0) & 0x3ff) << 22) >> 22) / d;
      accel_publisher_->msg_.y[i] = ((((acc >> 10) & 0x3ff) << 22) >> 22) / d;
      accel_publisher_->msg_.z[i] = ((((acc >> 20) & 0x3ff) << 22) >> 22) / d;
    }
    accel_publisher_->unlockAndPublish();
  }

  WG0X::convertState(state, current_buffer, last_buffer);
  last_pressure_time_ = p->timestamp_;
}

void WG0X::convertState(ActuatorState &state, unsigned char *this_buffer, unsigned char *prev_buffer)
{
  WG0XStatus *this_status, *prev_status;

  this_status = (WG0XStatus *)(this_buffer + command_size_);
  prev_status = (WG0XStatus *)(prev_buffer + command_size_);

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
  state.run_stop_hit_ = (this_status->mode_ & MODE_UNDERVOLTAGE) != 0;

  state.last_commanded_current_ = this_status->programmed_current_ * config_info_.nominal_current_scale_;
  state.last_measured_current_ = this_status->measured_current_ * config_info_.nominal_current_scale_;

  state.last_commanded_effort_ = this_status->programmed_current_ * config_info_.nominal_current_scale_ * actuator_info_.motor_torque_constant_ * actuator_info_.encoder_reduction_;
  state.last_measured_effort_ = this_status->measured_current_ * config_info_.nominal_current_scale_ * actuator_info_.motor_torque_constant_ * actuator_info_.encoder_reduction_;

  state.num_encoder_errors_ = this_status->num_encoder_errors_;
  state.num_communication_errors_ = 0; // TODO: communication errors are no longer reported in the process data

  state.motor_voltage_ = this_status->motor_voltage_ * config_info_.nominal_voltage_scale_;
}

bool WG0X::verifyState(ActuatorState &state, unsigned char *this_buffer, unsigned char *prev_buffer)
{
  bool rv = true;
  WG0XStatus *this_status, *prev_status;
  double expected_voltage;
  int level = 0;
  string reason = "OK";

  if (!(state.is_enabled_)) {
    goto end;
  }

  expected_voltage = state.last_measured_current_ * actuator_info_.resistance_ + state.velocity_ * actuator_info_.encoder_reduction_ * backemf_constant_;

  this_status = (WG0XStatus *)(this_buffer + command_size_);
  prev_status = (WG0XStatus *)(prev_buffer + command_size_);

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

  // Only read config info mailbox when we transition in/out of safety lockout
  if (bool(this_status->mode_ & MODE_SAFETY_LOCKOUT) != in_lockout_)
  {
      readMailbox(sh_, WG0XConfigInfo::CONFIG_INFO_BASE_ADDR, &config_info_, sizeof(config_info_));
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
  if(filtered_voltage_error_ > 5)
  {
    //Something is wrong with the encoder, the motor, or the motor board

    //Disable motors
    // TODO: don't disable motors for voltage error until all motor
    // parameters are properly configured
    //rv = false;

    const double epsilon = 0.001;
    //Try to diagnose further
    //motor_velocity == 0 -> encoder failure likely
    if (fabs(state.velocity_) < epsilon)
    {
      reason = "Encoder failure likely";
      level = 1;
    }
    //measured_current_ ~= 0 -> motor open-circuit likely
    else if (fabs(state.last_measured_current_) < epsilon)
    {
      reason = "Motor open-circuit likely";
      level = 1;
    }
    //motor_voltage_ ~= 0 -> motor short-circuit likely
    else if (fabs(voltage_estimate_) < epsilon)
    {
      reason = "Motor short-circuit likely";
      level = 1;
    }
    //else -> current-sense failure likely
    else
    {
      reason = "Current-sense failure likely";
      level = 1;
    }
  }

  //Check current-loop performance
  double last_commanded_current;
  last_commanded_current =  prev_status->programmed_current_ * config_info_.nominal_current_scale_;
  current_error_ = fabs(state.last_measured_current_ - last_commanded_current);

  max_current_error_ = max(current_error_, max_current_error_);

  if ((last_commanded_current > 0 ?
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
    reason = "Current loop error too large";
    level = 2;
  }

end:
  if (level_ != 1)
  {
    level_ = level;
    reason_ = reason;
  }
  return rv;
}

int WG0X::readData(EtherCAT_SlaveHandler *sh, EC_UINT address, void* buffer, EC_UINT length)
{
  unsigned char *p = (unsigned char *)buffer;
  EtherCAT_DataLinkLayer *dll = EtherCAT_DataLinkLayer::instance();
  EC_Logic *logic = EC_Logic::instance();

  // Build read telegram, use slave position
  APRD_Telegram status(logic->get_idx(), // Index
                       -sh->get_ring_position(), // Slave position on ethercat chain (auto increment address)
                       address, // ESC physical memory address (start address)
                       logic->get_wkc(), // Working counter
                       length, // Data Length,
                       p); // Buffer to put read result into

  // Put read telegram in ethercat/ethernet frame
  EC_Ethernet_Frame frame(&status);

  // Send/Recv data from slave
  if (!dll->txandrx(&frame))
  {
    status.set_wkc(logic->get_wkc());
    status.set_idx(logic->get_idx());
    if (!dll->txandrx(&frame))
    {
      return -1;
    }
  }

  // In some cases (clearing status mailbox) this is expected to occur
  if (status.get_wkc() != 1)
  {
    return -2;
  }

  return 0;
}

// Writes <length> amount of data from ethercat slave <sh_hub> from physical address <address> to <buffer>
int WG0X::writeData(EtherCAT_SlaveHandler *sh, EC_UINT address, void const* buffer, EC_UINT length)
{
  unsigned char *p = (unsigned char *)buffer;
  EtherCAT_DataLinkLayer *m_dll_instance = EtherCAT_DataLinkLayer::instance();
  EC_Logic *m_logic_instance = EC_Logic::instance();

  // Build write telegram, use slave position
  APWR_Telegram command(m_logic_instance->get_idx(), // Index
                        -sh->get_ring_position(), // Slave position on ethercat chain (auto increment address) (
                        address, // ESC physical memory address (start address)
                        m_logic_instance->get_wkc(), // Working counter
                        length, // Data Length,
                        p); // Buffer to put read result into

  // Put read telegram in ethercat/ethernet frame
  EC_Ethernet_Frame frame(&command);

  // Send/Recv data from slave
  if (!m_dll_instance->txandrx(&frame))
  {
    command.set_wkc(m_logic_instance->get_wkc());
    command.set_idx(m_logic_instance->get_idx());
    if (!m_dll_instance->txandrx(&frame))
    {
      return -1;
    }
  }

  if (command.get_wkc() != 1)
  {
    return -2;
  }

  return 0;
}

int WG0X::sendSpiCommand(EtherCAT_SlaveHandler *sh, WG0XSpiEepromCmd const * cmd)
{
  // Send command
  if (writeMailbox(sh, WG0XSpiEepromCmd::SPI_COMMAND_ADDR, cmd, sizeof(*cmd)))
  {
    fprintf(stderr, "ERROR WRITING EEPROM COMMAND\n");
    return -1;
  }

  for (int tries = 0; tries < 10; ++tries)
  {
    WG0XSpiEepromCmd stat;
    if (readMailbox(sh, WG0XSpiEepromCmd::SPI_COMMAND_ADDR, &stat, sizeof(stat)))
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

int WG0X::readEeprom(EtherCAT_SlaveHandler *sh)
{
  assert(sizeof(actuator_info_) == 264);
  WG0XSpiEepromCmd cmd;
  cmd.build_read(ACTUATOR_INFO_PAGE);
  if (sendSpiCommand(sh, &cmd)) {
    fprintf(stderr, "ERROR SENDING SPI EEPROM READ COMMAND\n");
    return -1;
  }
  // Read buffered data in multiple chunks
  if (readMailbox(sh, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, &actuator_info_, sizeof(actuator_info_))) {
    fprintf(stderr, "ERROR READING BUFFERED EEPROM PAGE DATA\n");
    return -1;
  }

  return 0;

}

void WG0X::program(WG0XActuatorInfo *info)
{

  writeMailbox(sh_, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, info, sizeof(WG0XActuatorInfo));
  WG0XSpiEepromCmd cmd;
  cmd.build_write(ACTUATOR_INFO_PAGE);
  if (sendSpiCommand(sh_, &cmd)) {
    fprintf(stderr, "ERROR SENDING SPI EEPROM WRITE COMMAND\n");
  }

  char data[2];
  memset(data, 0, sizeof(data));
  data[0] = 0xD7;

  if (writeMailbox(sh_, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, data, sizeof(data))) {
    fprintf(stderr, "ERROR WRITING EEPROM COMMAND BUFFER\n");
  }


  { // Start arbitrary command
    WG0XSpiEepromCmd cmd;
    cmd.build_arbitrary(sizeof(data));
    if (sendSpiCommand(sh_, &cmd)) {
      fprintf(stderr, "reading eeprom status failed");
    }
  }


  if (readMailbox(sh_, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, data, sizeof(data))) {
    fprintf(stderr, "ERROR READING EEPROM COMMAND BUFFER\n");
  }
}

int WG0X::readMailbox(EtherCAT_SlaveHandler *sh, int address, void *data, EC_UINT length)
{
  // first (re)read current status mailbox data to prevent issues with
  // the status mailbox being full (and unread) from last command
  WG0XMbxCmd stat;
  int result = readData(sh, MBX_STATUS_PHY_ADDR, &stat, sizeof(stat));

  if ((result != 0) && (result != -2))
  {
    fprintf(stderr, "CLEARING STATUS MBX FAILED result = %d\n", result);
    return -1;
  }

  // Build mailbox message and send read command
  WG0XMbxCmd cmd;
  cmd.build(address, length, false /*read*/, data);
  int tries;
  for (tries = 0; tries < 10; ++tries)
  {
    int result = writeData(sh, MBX_COMMAND_PHY_ADDR, &cmd, sizeof(cmd));
    if (result == -2)
    {
      // FPGA hasn't written responded with status data, wait a
      // tiny bit and try again.
      usleep(100); // 1/10th of a millisecond
      continue;
    }
    else if (result == 0)
    {
      // Successful read of status data
      break;
    }
    else
    {
      fprintf(stderr, "WRITING COMMAND MBX FAILED\n");
      return -1;
    }
  }
  if (tries >= 10)
  {
    fprintf(stderr, "do_mailbox_write : Too many tries writing mailbox\n");
    return -1;
  }

  for (tries = 0; tries < 10; ++tries)
  {
    int result = readData(sh, MBX_STATUS_PHY_ADDR, &stat, sizeof(stat));
    if (result == -2)
    {
      // FPGA hasn't written responded with status data, wait a
      // tiny bit and try again.
      usleep(100); // 1/10th of a millisecond
      continue;
    }
    else if (result == 0)
    {
      // Successfull read of status data
      break;
    }
    else
    {
      fprintf(stderr, "READING MBX STATUS FAILED\n");
      return -1;
    }
  }
  if (tries >= 10)
  {
    fprintf(stderr, "do_mailbox_read : Too many tries reading mailbox\n");
    return -1;
  }

  if (computeChecksum(&stat, length + 1) != 0)
  {
    fprintf(stderr, "CHECKSUM ERROR READING MBX DATA\n");
    return -1;
  }
  memcpy(data, &stat, length);
  return 0;
}

// Write <length> byte of <data> to <address> on FPGA local bus using the ethercat mailbox for communication
// Returns 0 for success and non-zero for failure.
int WG0X::writeMailbox(EtherCAT_SlaveHandler *sh, int address, void const *data, EC_UINT length)
{
  // Build mailbox message and write command
  {
    WG0XMbxCmd cmd;
    cmd.build(address, length, true /*write*/, data);
    int tries;
    for (tries = 0; tries < 10; ++tries)
    {
      int result = writeData(sh, MBX_COMMAND_PHY_ADDR, &cmd, sizeof(cmd));
      if (result == -2)
      {
        // FPGA hasn't written responded with status data, wait a
        // tiny bit and try again.
        usleep(100); // 1/10th of a millisecond
        continue;
      }
      else if (result == 0)
      {
        // Successfull read of status data
        return 0;
      }
      else
      {
        fprintf(stderr, "WRITING COMMAND MBX FAILED\n");
        return -1;
      }
    }
    if (tries >= 10)
    {
      fprintf(stderr, "do_mailbox_write : Too many tries writing mailbox\n");
      return -1;
    }
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

void WG0X::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  WG0XStatus *status = (WG0XStatus *)(buffer + command_size_);

  stringstream str;
  str << "EtherCAT Device (" << actuator_info_.name_ << ")";
  d.name = str.str();
  d.message = reason_;
  char serial[32];
  snprintf(serial, sizeof(serial), "%d-%05d-%05d", config_info_.product_id_ / 100000 , config_info_.product_id_ % 100000, config_info_.device_serial_number_);
  d.hardware_id = serial;
  d.level = level_;

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

  d.addf("Safety Disable Status", "%s (%02x)", safetyDisableString(config_info_.safety_disable_status_).c_str(), config_info_.safety_disable_status_);
  d.addf("Safety Disable Status Hold", "%s (%02x)", safetyDisableString(config_info_.safety_disable_status_hold_).c_str(), config_info_.safety_disable_status_hold_);
  d.addf("Safety Disable Count", "%d", config_info_.safety_disable_count_);
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
  d.addf("Timestamp", "%d", status->timestamp_);
  d.addf("Encoder count", "%d", status->encoder_count_);
  d.addf("Encoder index pos", "%d", status->encoder_index_pos_);
  d.addf("Num encoder_errors", "%d", status->num_encoder_errors_);
  d.addf("Encoder status", "%d", status->encoder_status_);
  d.addf("Calibration reading", "%d", status->calibration_reading_);
  d.addf("Last calibration rising edge", "%d", status->last_calibration_rising_edge_);
  d.addf("Last calibration falling edge", "%d", status->last_calibration_falling_edge_);
  d.addf("Board temperature", "%f", 0.0078125 * status->board_temperature_);
  d.addf("Bridge temperature", "%f", 0.0078125 * status->bridge_temperature_);
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
}
