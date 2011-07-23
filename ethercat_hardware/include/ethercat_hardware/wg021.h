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

#ifndef ETHERCAT_HARDWARE_WG021_H
#define ETHERCAT_HARDWARE_WG021_H

#include <ethercat_hardware/wg0x.h>


struct WG021Status
{
  uint8_t mode_;
  uint8_t digital_out_;
  uint8_t general_config_;
  uint8_t pad1_;
  int16_t programmed_current_;
  int16_t measured_current_;
  uint32_t timestamp_;
  uint8_t config0_;
  uint8_t config1_;
  uint8_t config2_;
  uint8_t pad2_;
  uint32_t pad3_;
  uint16_t pad4_;
  uint8_t pad5_;
  uint8_t output_status_;
  uint32_t output_start_timestamp_;
  uint32_t output_stop_timestamp_;
  uint16_t board_temperature_;
  uint16_t bridge_temperature_;
  uint16_t supply_voltage_;
  int16_t led_voltage_;
  uint16_t packet_count_;
  uint8_t pad_;
  uint8_t checksum_;
  static const unsigned SIZE=44;
}__attribute__ ((__packed__));

struct WG021Command
{
  uint8_t mode_;
  uint8_t digital_out_;
  uint8_t general_config_;
  uint8_t pad1_;
  int16_t programmed_current_;
  int16_t pad2_;
  int32_t pad3_;
  uint8_t config0_;
  uint8_t config1_;
  uint8_t config2_;
  uint8_t checksum_;
}__attribute__ ((__packed__));

class WG021 : public WG0X
{
public:
  WG021() : projector_(digital_out_A_, digital_out_B_, digital_out_I_, digital_out_M_, digital_out_L0_, digital_out_L1_) {}
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  int initialize(pr2_hardware_interface::HardwareInterface *, bool allow_unprogrammed=true);
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  enum
  {
    PRODUCT_CODE = 6805021
  };
  enum
  {
    PROJECTOR_CONFIG_ENABLE = 8,
    PROJECTOR_CONFIG_ENABLE_ENABLED = 8,
    PROJECTOR_CONFIG_ENABLE_DISABLED = 0,

    PROJECTOR_CONFIG_ACTION = 4,
    PROJECTOR_CONFIG_ACTION_ON = 4,
    PROJECTOR_CONFIG_ACTION_OFF = 0,

    PROJECTOR_CONFIG_POLARITY = 2,
    PROJECTOR_CONFIG_POLARITY_ACTIVE_HIGH = 2,
    PROJECTOR_CONFIG_POLARITY_ACTIVE_LOW = 0,

    PROJECTOR_CONFIG_STATE = 1,
    PROJECTOR_CONFIG_STATE_HIGH = 1,
    PROJECTOR_CONFIG_STATE_LOW = 0
  };
private:
  pr2_hardware_interface::DigitalOut digital_out_A_;
  pr2_hardware_interface::DigitalOut digital_out_B_;
  pr2_hardware_interface::DigitalOut digital_out_I_;
  pr2_hardware_interface::DigitalOut digital_out_M_;
  pr2_hardware_interface::DigitalOut digital_out_L0_;
  pr2_hardware_interface::DigitalOut digital_out_L1_;
  pr2_hardware_interface::Projector projector_;
};

#endif /* ETHERCAT_HARDWARE_WG021_H */
