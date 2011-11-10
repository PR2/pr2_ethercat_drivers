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


#include "ethercat_hardware/ethercat_com.h"
#include "ethercat_hardware/mailbox.h"
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

namespace ethercat_hardware
{

enum MbxCmdType {LOCAL_BUS_READ=1, LOCAL_BUS_WRITE=2};

struct WG0XMbxHdr
{
  uint16_t address_;
  union
  {
    uint16_t command_;
    struct
    {
      uint16_t length_:12;
      uint16_t seqnum_: 3;  // bits[14:12] sequence number, 0=disable, 1-7 normal sequence number
      uint16_t write_nread_:1;
    }__attribute__ ((__packed__));
  };
  uint8_t checksum_;

  bool build(unsigned address, unsigned length, MbxCmdType type, unsigned seqnum);
  bool verifyChecksum(void) const;
}__attribute__ ((__packed__));

static const unsigned MBX_SIZE = 512;
static const unsigned MBX_DATA_SIZE = (MBX_SIZE - sizeof(WG0XMbxHdr) - 1);
struct WG0XMbxCmd
{
  WG0XMbxHdr hdr_;
  uint8_t data_[MBX_DATA_SIZE];
  uint8_t checksum_;

  bool build(unsigned address, unsigned length, MbxCmdType type, unsigned seqnum, void const* data);
}__attribute__ ((__packed__));



struct MbxDiagnostics 
{
  MbxDiagnostics();
  uint32_t write_errors_;
  uint32_t read_errors_;
  uint32_t lock_errors_;
  uint32_t retries_;
  uint32_t retry_errors_;
};


class WGMailbox : public Mailbox
{
public:
  WGMailbox();

  bool write(EthercatCom *com, unsigned address, void const *data, unsigned length);
  bool read(EthercatCom *com, unsigned address, void *data, unsigned length);

  bool initialize(EtherCAT_SlaveHandler *sh);
  int writeMailbox(EthercatCom *com, unsigned address, void const *data, unsigned length);
  int readMailbox(EthercatCom *com, unsigned address, void *data, unsigned length);
  void publishMailboxDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);

  static const unsigned MBX_COMMAND_PHY_ADDR = 0x1400;
  static const unsigned MBX_COMMAND_SIZE = 512;
  static const unsigned MBX_STATUS_PHY_ADDR = 0x2400;
  static const unsigned MBX_STATUS_SIZE = 512;

  static const unsigned MBX_COMMAND_SYNCMAN_NUM = 2;
  static const unsigned MBX_STATUS_SYNCMAN_NUM  = 3;

protected:
  // Each WG0X device can only support one mailbox operation at a time
  bool lockMailbox();
  void unlockMailbox();
  pthread_mutex_t mailbox_lock_;
  MbxDiagnostics mailbox_diagnostics_;
  MbxDiagnostics mailbox_publish_diagnostics_;

  // Mailbox helper functions
  int writeMailbox_(EthercatCom *com, unsigned address, void const *data, unsigned length);
  int readMailbox_(EthercatCom *com, unsigned address, void *data, unsigned length);  
  bool verifyDeviceStateForMailboxOperation();
  bool clearReadMailbox(EthercatCom *com);
  bool waitForReadMailboxReady(EthercatCom *com);
  bool waitForWriteMailboxReady(EthercatCom *com);
  bool readMailboxRepeatRequest(EthercatCom *com);
  bool _readMailboxRepeatRequest(EthercatCom *com);
  bool writeMailboxInternal(EthercatCom *com, void const *data, unsigned length);
  bool readMailboxInternal(EthercatCom *com, void *data, unsigned length);
  void diagnoseMailboxError(EthercatCom *com);
  
  EtherCAT_SlaveHandler *sh_;
};


}; //end namespace ethercat_hardware
