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

#include "ethercat_hardware/wg_util.h"

namespace ethercat_hardware
{


unsigned int wg_util::rotateRight8(unsigned in)
{
  in &= 0xff;
  in = (in >> 1) | (in << 7);
  in &= 0xff;
  return in;
}

unsigned wg_util::computeChecksum(void const *data, unsigned length)
{
  const unsigned char *d = (const unsigned char *)data;
  unsigned int checksum = 0x42;
  for (unsigned int i = 0; i < length; ++i)
  {
    checksum = rotateRight8(checksum);
    checksum ^= d[i];
    checksum &= 0xff;
  }
  return checksum;
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


}; //end namespace ethercat_hardware
