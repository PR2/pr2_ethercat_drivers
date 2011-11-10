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

#pragma once

#include "ethercat_hardware/wg_mailbox.h"
#include "ethercat_hardware/ethercat_com.h"
#include <stdint.h>
#include <boost/thread/mutex.hpp>

namespace ethercat_hardware
{


struct WG0XSpiEepromCmd
{
  uint16_t page_;
  union
  {
    uint8_t command_;
    struct
    {
      uint8_t operation_ :4;
      uint8_t start_ :1;
      uint8_t busy_ :1;
      uint8_t unused2_ :2;
    }__attribute__ ((__packed__));
  };

  void build_read(unsigned page)
  {
    this->page_ = page & 0xffff;
    this->operation_ = SPI_READ_OP;
    this->start_ = 1;
  }
  void build_write(unsigned page)
  {
    this->page_ = page & 0xffff;
    this->operation_ = SPI_WRITE_OP;
    this->start_ = 1;
  }
  void build_arbitrary(unsigned length)
  {
    this->page_ = (length-1) & 0xffff;
    this->operation_ = SPI_ARBITRARY_OP;
    this->start_ = 1;
  }

  static const unsigned SPI_READ_OP = 0;
  static const unsigned SPI_WRITE_OP = 1;
  static const unsigned SPI_ARBITRARY_OP = 3;

  static const unsigned SPI_COMMAND_ADDR = 0x0230;
  static const unsigned SPI_BUFFER_ADDR = 0xF400;
}__attribute__ ((__packed__));



struct EepromStatusReg 
{
  union {
    uint8_t raw_;
    struct {
      uint8_t page_size_     : 1; 
      uint8_t write_protect_ : 1;
      uint8_t eeprom_size_   : 4;
      uint8_t compare_       : 1;
      uint8_t ready_         : 1;
    } __attribute__ ((__packed__));
  } __attribute__ ((__packed__));
} __attribute__ ((__packed__));


class WGEeprom
{
public:
  WGEeprom();
  bool readEepromPage(EthercatCom *com, WGMailbox *mbx, unsigned page, void* data, unsigned length);
  bool writeEepromPage(EthercatCom *com, WGMailbox *mbx, unsigned page, const void* data, unsigned length);  

protected:
  static const unsigned NUM_EEPROM_PAGES   = 4096;
  static const unsigned MAX_EEPROM_PAGE_SIZE = 264;

  // SPI Eeprom State machine helper functions
  bool readSpiEepromCmd(EthercatCom *com, WGMailbox *mbx, WG0XSpiEepromCmd &cmd);
  bool sendSpiEepromCmd(EthercatCom *com, WGMailbox *mbx, const WG0XSpiEepromCmd &cmd);
  bool waitForSpiEepromReady(EthercatCom *com, WGMailbox *mbx);

  // Eeprom helper functions
  bool readEepromStatusReg(EthercatCom *com, WGMailbox *mbx, EepromStatusReg &reg);
  bool waitForEepromReady(EthercatCom *com, WGMailbox *mbx);

  //! Only one eeprom transaction can occur at a time
  boost::mutex mutex_;
};


}; //end namespace ethercat_hardware
