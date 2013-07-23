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

#include <ethercat_hardware/wg05.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <boost/crc.hpp>
#include <boost/static_assert.hpp>

PLUGINLIB_EXPORT_CLASS(WG05, EthercatDevice);


void WG05::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  WG0X::construct(sh, start_address);

  unsigned int base_status = sizeof(WG0XStatus);

  // As good a place as any for making sure that compiler actually packed these structures correctly
  BOOST_STATIC_ASSERT(sizeof(WG0XStatus) == WG0XStatus::SIZE);

  command_size_ = sizeof(WG0XCommand);
  status_size_ = sizeof(WG0XStatus);


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


int WG05::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  if ((fw_major_ == 1) && (fw_minor_ >= 21)) 
  {
    app_ram_status_ = APP_RAM_PRESENT;
  }

  int retval = WG0X::initialize(hw, allow_unprogrammed);

  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());

  // Determine if device supports application RAM
  if (!retval)
  {
    if (use_ros_)
    {
      // WG005B has very poor motor voltage measurement, don't use meaurement for dectecting problems. 
      bool poor_measured_motor_voltage = (board_major_ <= 2);
      double max_pwm_ratio = double(0x3C00) / double(PWM_MAX);
      double board_resistance = 0.8;
      if (!WG0X::initializeMotorModel(hw, "WG005", max_pwm_ratio, board_resistance, poor_measured_motor_voltage)) 
      {
        ROS_FATAL("Initializing motor trace failed");
        sleep(1); // wait for ros to flush rosconsole output
        return -1;
      }
    }

  }// end if !retval
  return retval;
}

void WG05::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  WG0X::packCommand(buffer, halt, reset);
}

bool WG05::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  bool rv = true;

  unsigned char* this_status = this_buffer + command_size_;
  if (!verifyChecksum(this_status, status_size_))
  {
    status_checksum_error_  = true;
    rv = false;
    goto end;
  }

  if (!WG0X::unpackState(this_buffer, prev_buffer))
  {
    rv = false;
  }

 end:
  return rv;
}


