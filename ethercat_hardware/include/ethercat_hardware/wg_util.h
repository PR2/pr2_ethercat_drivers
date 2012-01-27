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

#include <stdint.h>
#include "ethercat_hardware/ethercat_com.h"
#include "ethercat_hardware/ethercat_device.h"

namespace ethercat_hardware
{

// Syncmanger control register 0x804+N*8
struct SyncManControl {
  union {
    uint8_t raw;
    struct {
      uint8_t mode            : 2;
      uint8_t direction       : 2;
      uint8_t ecat_irq_enable : 1;
      uint8_t pdi_irq_enable  : 1;
      uint8_t watchdog_enable : 1;
      uint8_t res1            : 1;
    } __attribute__ ((__packed__));      
  } __attribute__ ((__packed__));
  //static const unsigned BASE_ADDR=0x804;
  //static unsigned base_addr(unsigned num);
  //void print(std::ostream &os=std::cout) const;
} __attribute__ ((__packed__));

// Syncmanger status register 0x805+N*8
struct SyncManStatus {
  union {
    uint8_t raw;
    struct {
      uint8_t interrupt_write : 1;
      uint8_t interrupt_read  : 1;
      uint8_t res1            : 1;
      uint8_t mailbox_status  : 1;
      uint8_t buffer_status   : 2;
      uint8_t res2            : 2;
    } __attribute__ ((__packed__));      
  } __attribute__ ((__packed__));
  //static const unsigned BASE_ADDR=0x805;
  //static unsigned base_addr(unsigned num);
  //void print(std::ostream &os=std::cout) const;
} __attribute__ ((__packed__));

// Syncmanger activation register 0x806+N*8
struct SyncManActivate {
  union {
    uint8_t raw;
    struct {
      uint8_t enable : 1;
      uint8_t repeat_request : 1;
      uint8_t res4 : 4;
      uint8_t ecat_latch_event : 1;
      uint8_t pdi_latch_event : 1;
    } __attribute__ ((__packed__));      
  } __attribute__ ((__packed__));
  static const unsigned BASE_ADDR=0x806;
  static unsigned baseAddress(unsigned num);
  //void print(std::ostream &os=std::cout) const;  
  bool writeData(EthercatCom *com, EtherCAT_SlaveHandler *sh, EthercatDevice::AddrMode addrMode, unsigned num) const;
} __attribute__ ((__packed__));

// Syncmanger PDI control register 0x807+N*8
struct SyncManPDIControl {
  union {
    uint8_t raw;
    struct {
      uint8_t deactivate : 1;
      uint8_t repeat_ack : 1;
      uint8_t res6 : 6;
    } __attribute__ ((__packed__));
  } __attribute__ ((__packed__));      
  //static const unsigned BASE_ADDR=0x807;
  //static unsigned base_addr(unsigned num);
  //void print(std::ostream &os=std::cout) const;
} __attribute__ ((__packed__));


// For SyncManager settings REG 0x800+8*N
struct SyncMan {
  union {
    uint8_t raw[8];
    struct {
      uint16_t start_addr;
      uint16_t length;
      SyncManControl control;
      SyncManStatus status;
      SyncManActivate activate;
      SyncManPDIControl pdi_control;
    } __attribute__ ((__packed__));
  } __attribute__ ((__packed__));
  
  // Base address for first syncmanager
  static const unsigned BASE_ADDR=0x800;
  // Base address of Nth syncmanager for N=0-7
  static unsigned baseAddress(unsigned num);
  
  bool readData(EthercatCom *com, EtherCAT_SlaveHandler *sh, EthercatDevice::AddrMode addrMode, unsigned num);
  //void print(unsigned num, std::ostream &os=std::cout) const;
} __attribute__ ((__packed__));

namespace wg_util
{
unsigned computeChecksum(void const *data, unsigned length);
unsigned int rotateRight8(unsigned in);
};

}; // end namespace ethercat_hardware
