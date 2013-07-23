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

#include <ethercat_hardware/wg014.h>
#include <iomanip>

#include <ros/console.h>

PLUGINLIB_EXPORT_CLASS(WG014, EthercatDevice);

void WG014::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  EthercatDevice::construct(sh, start_address);
  sh->set_fmmu_config( new EtherCAT_FMMU_Config(0) );
  sh->set_pd_config( new EtherCAT_PD_Config(0) );

  fw_major_ = (sh->get_revision() >> 8) & 0xff;
  fw_minor_ = sh->get_revision() & 0xff;
  board_major_ = ((sh->get_revision() >> 24) & 0xff) - 1;
  board_minor_ = (sh->get_revision() >> 16) & 0xff;
}

WG014::~WG014()
{
  delete sh_->get_fmmu_config();
  delete sh_->get_pd_config();
}

int WG014::initialize(pr2_hardware_interface::HardwareInterface *, bool)
{
  ROS_DEBUG("Device #%02d: WG014 (%#08x)", sh_->get_ring_position(), sh_->get_product_code());
  return 0;
}

void WG014::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *)
{
  stringstream str;
  str << "EtherCAT Device #" << setw(2) << setfill('0') << sh_->get_ring_position() << " (WG014)";
  d.name = str.str();
  d.summary(0, "OK");
  char serial[32];
  snprintf(serial, sizeof(serial), "%d-%05d-%05d", sh_->get_product_code()/ 100000 , sh_->get_product_code() % 100000, sh_->get_serial());
  d.hardware_id = serial;

  d.clear();
  d.addf("Product code", "WG014 (%d), Ports %s, PCB Revision %c.%02d",
         sh_->get_product_code(), 
         ((fw_minor_==1) ? "J1-J3" : // WG014 has two ET1100 devices...
          (fw_minor_==2) ? "J4-J6" : // One ET1100 handles port J1 to J3. The other, J4 to J6.
          "J?-J?"),                  // The firmware minor revision deferentiates the two.
         'A' + board_major_, board_minor_);
  d.addf("Serial Number", "%s", serial);

  EthercatDevice::ethercatDiagnostics(d, 4); // WG014 has 4 ports
}
