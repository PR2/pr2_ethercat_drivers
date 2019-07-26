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
#include <stddef.h>

#include <ethercat_hardware/wg021.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <boost/static_assert.hpp>

#include "ethercat_hardware/wg_util.h"

PLUGINLIB_EXPORT_CLASS(WG021, EthercatDevice);

void WG021::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  WG0X::construct(sh, start_address);

  unsigned int base_status = sizeof(WG0XStatus);

  // As good a place as any for making sure that compiler actually packed these structures correctly
  BOOST_STATIC_ASSERT(sizeof(WG021Status) == WG021Status::SIZE);

  status_size_ = base_status = sizeof(WG021Status);
  command_size_ = sizeof(WG021Command);

  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);
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
                       base_status, // Logical length
                       0x00, // Logical StartBit
                       0x07, // Logical EndBit
                       STATUS_PHY_ADDR, // Physical Start address
                       0x00, // Physical StartBit
                       true, // Read Enable
                       false, // Write Enable
                       true); // Enable

  start_address += base_status;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(4);

  // Sync managers
  (*pd)[0] = EC_SyncMan(COMMAND_PHY_ADDR, command_size_, EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;

  (*pd)[1] = EC_SyncMan(STATUS_PHY_ADDR, base_status);
  (*pd)[1].ChannelEnable = true;

  (*pd)[2] = EC_SyncMan(WGMailbox::MBX_COMMAND_PHY_ADDR, WGMailbox::MBX_COMMAND_SIZE, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
  (*pd)[2].ChannelEnable = true;
  (*pd)[2].ALEventEnable = true;

  (*pd)[3] = EC_SyncMan(WGMailbox::MBX_STATUS_PHY_ADDR, WGMailbox::MBX_STATUS_SIZE, EC_QUEUED);
  (*pd)[3].ChannelEnable = true;

  sh->set_pd_config(pd);
}

int WG021::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  // WG021 has no use for application ram
  app_ram_status_ = APP_RAM_NOT_APPLICABLE;

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
        return -1;
    }
  }

  // Register projector with pr2_hardware_interface::HardwareInterface
  {
    projector_.name_ = actuator_info_.name_;
    if (hw && !hw->addProjector(&projector_))
    {
        ROS_FATAL("A projector of the name '%s' already exists.  Device #%02d has a duplicate name", projector_.name_.c_str(), sh_->get_ring_position());
        return -1;
    }
    projector_.command_.enable_ = true;
    projector_.command_.current_ = 0;
  }

  return retval;
}

void WG021::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  pr2_hardware_interface::ProjectorCommand &cmd = projector_.command_;

  // Override enable if motors are halted  
  if (reset) 
  {
    clearErrorFlags();
  }
  resetting_ = reset;

  // Truncate the current to limit (do not allow negative current)
  projector_.state_.last_commanded_current_ = cmd.current_;
  cmd.current_ = max(min(cmd.current_, max_current_), 0.0);

  // Pack command structures into EtherCAT buffer
  WG021Command *c = (WG021Command *)buffer;
  memset(c, 0, command_size_);
  c->digital_out_ = digital_out_.command_.data_;
  c->programmed_current_ = int(cmd.current_ / config_info_.nominal_current_scale_);
  c->mode_ = (cmd.enable_ && !halt && !has_error_) ? (MODE_ENABLE | MODE_CURRENT) : MODE_OFF;
  c->mode_ |= reset ? MODE_SAFETY_RESET : 0;
  c->config0_ = ((cmd.A_ & 0xf) << 4) | ((cmd.B_ & 0xf) << 0);
  c->config1_ = ((cmd.I_ & 0xf) << 4) | ((cmd.M_ & 0xf) << 0);
  c->config2_ = ((cmd.L1_ & 0xf) << 4) | ((cmd.L0_ & 0xf) << 0);
  c->general_config_ = cmd.pulse_replicator_ == true;
  c->checksum_ = wg_util::rotateRight8(wg_util::computeChecksum(c, command_size_ - 1));
}

bool WG021::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  bool rv = true;

  pr2_hardware_interface::ProjectorState &state = projector_.state_;
  WG021Status *this_status;
  //WG021Status *prev_status;
  this_status = (WG021Status *)(this_buffer + command_size_);
  //prev_status = (WG021Status *)(prev_buffer + command_size_);
  
  if (!verifyChecksum(this_status, status_size_))
  {
    status_checksum_error_ = true;
    rv = false;
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
  state.L1_ = ((this_status->config2_ >> 4) & 0xf);
  state.L0_ = ((this_status->config2_ >> 0) & 0xf);
  state.pulse_replicator_ = (this_status->general_config_ & 0x1) == 0x1;

  state.last_executed_current_ = this_status->programmed_current_ * config_info_.nominal_current_scale_;
  state.last_measured_current_ = this_status->measured_current_ * config_info_.nominal_current_scale_;

  state.max_current_ = max_current_;

  max_board_temperature_ = max(max_board_temperature_, this_status->board_temperature_);
  max_bridge_temperature_ = max(max_bridge_temperature_, this_status->bridge_temperature_);

  if (!verifyState((WG0XStatus *)(this_buffer + command_size_), (WG0XStatus *)(prev_buffer + command_size_)))
  {
    rv = false;
  }

end:
  return rv;
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

  d.summary(d.OK, "OK");

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
  d.addf("SW Max Current", "%f", actuator_info_.max_current_);

  publishGeneralDiagnostics(d);
  mailbox_.publishMailboxDiagnostics(d);

  d.add("Mode", modeString(status->mode_));
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
