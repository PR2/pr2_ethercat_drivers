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

#ifndef ETHERCAT_HARDWARE__WG_SOFT_PROCESSOR_H
#define ETHERCAT_HARDWARE__WG_SOFT_PROCESSOR_H

#include <ros/ros.h>
#include "ethercat_hardware/ethercat_com.h"
#include "ethercat_hardware/wg_mailbox.h"
#include "ethercat_hardware/SoftProcessorFirmwareWrite.h"
#include "ethercat_hardware/SoftProcessorFirmwareRead.h"
#include "ethercat_hardware/SoftProcessorReset.h"

#include <vector>
#include <ostream>
#include <string>

namespace ethercat_hardware
{


/**  Certain version of WG0X device firmware use soft proccessors to perform certain tasks.
* For soft processors it is sometimes benificial to allow firmware to be modified 
* to support different devices
*/
class WGSoftProcessor
{
public:
  WGSoftProcessor();
  bool initialize(EthercatCom *com);

  struct Info 
  {
    Info() : mbx_(NULL), iram_address_(-1), ctrl_address_(-1) {}
    Info( WGMailbox *mbx,
          const std::string &actuator_name, 
          const std::string &processor_name, 
          unsigned iram_address, unsigned ctrl_address ) :
      mbx_(mbx), actuator_name_(actuator_name), processor_name_(processor_name),
      iram_address_(iram_address), ctrl_address_(ctrl_address) {}
    WGMailbox *mbx_;    
    std::string actuator_name_;
    std::string processor_name_;
    unsigned iram_address_;
    unsigned ctrl_address_;
  };

  void add(WGMailbox *mbx, const std::string &actuator_name, const std::string &processor_name, 
           unsigned iram_address, unsigned ctrl_address);  

protected:

  static const unsigned IRAM_INSTRUCTION_LENGTH = 1024;

  std::vector<Info> processors_;

  bool writeFirmwareCB(ethercat_hardware::SoftProcessorFirmwareWrite::Request &request, 
                       ethercat_hardware::SoftProcessorFirmwareWrite::Response &response);
  bool readFirmwareCB(ethercat_hardware::SoftProcessorFirmwareRead::Request &request, 
                      ethercat_hardware::SoftProcessorFirmwareRead::Response &response);
  bool resetCB(ethercat_hardware::SoftProcessorReset::Request &request, 
               ethercat_hardware::SoftProcessorReset::Response &response);

  ros::ServiceServer read_firmware_service_; //!< service that allows read of soft processor firmware
  ros::ServiceServer write_firmware_service_; //!< service that allows write of soft processor firmware
  ros::ServiceServer reset_service_;  //!< service that resets soft-processor

  //! Puts soft processor in reset
  bool assertReset(const Info &info, std::ostream &err_msg);

  //! Takes soft processor out of reset
  bool releaseReset(const Info &info, std::ostream &err_msg);

  //! Get pointer to soft processor by name. Returns NULL if processor d/n exist and create message in err_out
  const WGSoftProcessor::Info* get(const std::string &actuator_name, const std::string &processor_name, std::ostream &err_out) const;
    
  EthercatCom *com_;
};


};  // end namespace ethercat_hardware

#endif //ETHERCAT_HARDWARE__WG_SOFT_PROCESSOR_H
