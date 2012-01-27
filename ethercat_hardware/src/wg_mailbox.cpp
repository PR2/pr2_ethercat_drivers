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

#include "ethercat_hardware/wg_mailbox.h"
#include "ethercat_hardware/wg_util.h"
#include "dll/ethercat_device_addressed_telegram.h"
#include "ethercat_hardware/ethercat_device.h"

namespace ethercat_hardware
{

// Temporary,, need 'log' fuction that can switch between fprintf and ROS_LOG.
#define ERR_MODE "\033[41m"
#define STD_MODE "\033[0m"
#define WARN_MODE "\033[43m"
#define GOOD_MODE "\033[42m"
#define INFO_MODE "\033[44m"

#define ERROR_HDR "\033[41mERROR\033[0m"
#define WARN_HDR "\033[43mERROR\033[0m"



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



bool WG0XMbxHdr::build(unsigned address, unsigned length, MbxCmdType type, unsigned seqnum)
{
  if (type==LOCAL_BUS_WRITE) 
  {
    if (length > MBX_DATA_SIZE) 
    {
      fprintf(stderr, "size of %d is too large for write\n", length);
      return false;
    }
  }
  else if (type==LOCAL_BUS_READ) 
  {
    // Result of mailbox read, only stores result data + 1byte checksum
    if (length > (MBX_SIZE-1))
    {
      fprintf(stderr, "size of %d is too large for read\n", length);
      return false;      
    }
  }
  else {
    assert(0 && "invalid MbxCmdType");
    return false;
  }
  
  address_ = address;
  length_ = length - 1;
  seqnum_ = seqnum;
  write_nread_ = (type==LOCAL_BUS_WRITE) ? 1 : 0;
  checksum_ = wg_util::rotateRight8(wg_util::computeChecksum(this, sizeof(*this) - 1));
  return true;
}

bool WG0XMbxHdr::verifyChecksum(void) const
{
  return wg_util::computeChecksum(this, sizeof(*this)) != 0;
}

bool WG0XMbxCmd::build(unsigned address, unsigned length, MbxCmdType type, unsigned seqnum, void const* data)
{
  if (!this->hdr_.build(address, length, type, seqnum))
  {
    return false;
  }
      
  if (data != NULL)
  {
    memcpy(data_, data, length);
  }
  else
  {
    memset(data_, 0, length);
  }
  unsigned int checksum = wg_util::rotateRight8(wg_util::computeChecksum(data_, length));
  data_[length] = checksum;
  return true;
}


MbxDiagnostics::MbxDiagnostics() :
  write_errors_(0),
  read_errors_(0),
  lock_errors_(0),
  retries_(0),
  retry_errors_(0)
{
  // Empty
}


/*!
 * \brief  Find difference between two timespec values
 *
 * \param current   current time 
 * \param current   start time 
 * \return          returns time difference (current-start) in milliseconds
 */
int timediff_ms(const timespec &current, const timespec &start)
{
  int timediff_ms = (current.tv_sec-start.tv_sec)*1000 // 1000 ms in a sec
    + (current.tv_nsec-start.tv_nsec)/1000000; // 1000000 ns in a ms
  return timediff_ms;
}


/*!
 * \brief  error checking wrapper around clock_gettime
 *
 * \param current   current time 
 * \param current   start time 
 * \return          returns 0 for success, non-zero for failure
 */
int safe_clock_gettime(clockid_t clk_id, timespec *time)
{
  int result = clock_gettime(clk_id, time);
  if (result != 0) {
    int error = errno;
    fprintf(stderr, "safe_clock_gettime : %s\n", strerror(error));
    return result;
  }  
  return result;
}


/*!
 * \brief  safe version of usleep.
 *
 * Uses nanosleep internally.  Will restart sleep after begin woken by signal.
 *
 * \param usec   number of microseconds to sleep for.  Must be < 1000000.
 */
void safe_usleep(uint32_t usec) 
{
  assert(usec<1000000);
  if (usec>1000000)
    usec=1000000;
  struct timespec req, rem;
  req.tv_sec = 0;
  req.tv_nsec = usec*1000;
  while (nanosleep(&req, &rem)!=0) { 
    int error = errno;
    fprintf(stderr,"%s : Error : %s\n", __func__, strerror(error));    
    if (error != EINTR) {
      break;
    }
    req = rem;
  }
  return;
}




void updateIndexAndWkc(EC_Telegram *tg, EC_Logic *logic) 
{
  tg->set_idx(logic->get_idx());
  tg->set_wkc(logic->get_wkc());
}

WGMailbox::WGMailbox() : sh_(NULL)
{
  int error;
  if ((error = pthread_mutex_init(&mailbox_lock_, NULL)) != 0)
  {
    ROS_ERROR("WG0X : init mailbox mutex :%s", strerror(error));
  }
}

bool WGMailbox::initialize(EtherCAT_SlaveHandler *sh)
{
  sh_ = sh;
  return true;
}

bool WGMailbox::verifyDeviceStateForMailboxOperation()
{
  // Make sure slave is in correct state to do use mailbox
  EC_State state = sh_->get_state();
  if ((state != EC_SAFEOP_STATE) && (state != EC_OP_STATE)) {
    fprintf(stderr, "%s : " ERROR_HDR 
            "cannot do mailbox read in current device state = %d\n", __func__, state);
    return false;
  }
  return true;
}


/*!
 * \brief  Runs diagnostic on read and write mailboxes.
 *
 * Collects and data from mailbox control registers.
 *
 * \todo            not implemented yet
 * \param com       used to perform communication with device
 * \return          returns true for success, false for failure 
 */
void WGMailbox::diagnoseMailboxError(EthercatCom *com)
{
  
}

/*!
 * \brief  Clears read mailbox by reading first and last byte.
 *
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \return          returns true for success, false for failure 
 */
bool WGMailbox::clearReadMailbox(EthercatCom *com)
{
  if (!verifyDeviceStateForMailboxOperation()){
    return false;
  }

  EC_Logic *logic = EC_Logic::instance();    
  EC_UINT station_addr = sh_->get_station_address();  
  
  // Create Ethernet packet with two EtherCAT telegrams inside of it : 
  //  - One telegram to read first byte of mailbox
  //  - One telegram to read last byte of mailbox
  unsigned char unused[1] = {0};
  NPRD_Telegram read_start(
            logic->get_idx(),
            station_addr,
            MBX_STATUS_PHY_ADDR,
            logic->get_wkc(),
            sizeof(unused),
            unused);
  NPRD_Telegram read_end(  
            logic->get_idx(),
            station_addr,
            MBX_STATUS_PHY_ADDR+MBX_STATUS_SIZE-1,
            logic->get_wkc(),
            sizeof(unused),
             unused);
  read_start.attach(&read_end);
  EC_Ethernet_Frame frame(&read_start);


  // Retry sending packet multiple times 
  bool success=false;
  static const unsigned MAX_DROPS = 15;
  for (unsigned tries=0; tries<MAX_DROPS; ++tries) {
    success = com->txandrx_once(&frame);
    if (success) {
      break;
    }
    updateIndexAndWkc(&read_start, logic);
    updateIndexAndWkc(&read_end  , logic);
  }

  if (!success) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " too much packet loss\n", __func__);   
    safe_usleep(100);
    return false;
  }
  
  // Check result for consistancy
  if (read_start.get_wkc() != read_end.get_wkc()) {
    fprintf(stderr, "%s : " ERROR_HDR 
            "read mbx working counters are inconsistant, %d, %d\n",
            __func__, read_start.get_wkc(), read_end.get_wkc());
    return false;
  }
  if (read_start.get_wkc() > 1) {
    fprintf(stderr, "%s : " ERROR_HDR 
            "more than one device (%d) responded \n", __func__, read_start.get_wkc());
    return false;
  }
  if (read_start.get_wkc() == 1)  {
    fprintf(stderr, "%s : " WARN_MODE "WARN" STD_MODE 
            " read mbx contained garbage data\n", __func__);
    // Not an error, just warning
  } 
  
  return true;  
}



/*!
 * \brief  Waits until read mailbox is full or timeout.
 *
 * Wait times out after 100msec.
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \return          returns true for success, false for failure or timeout
 */
bool WGMailbox::waitForReadMailboxReady(EthercatCom *com)
{
  // Wait upto 100ms for device to toggle ack
  static const int MAX_WAIT_TIME_MS = 100;
  int timediff;
  unsigned good_results=0;


  struct timespec start_time, current_time;
  if (safe_clock_gettime(CLOCK_MONOTONIC, &start_time)!=0) {
    return false;
  }
  
  do {      
    // Check if mailbox is full by looking at bit 3 of SyncMan status register.
    uint8_t SyncManStatus=0;
    const unsigned SyncManAddr = 0x805+(MBX_STATUS_SYNCMAN_NUM*8);
    if (EthercatDevice::readData(com, sh_, SyncManAddr, &SyncManStatus, sizeof(SyncManStatus), EthercatDevice::FIXED_ADDR) == 0) {
      ++good_results;
      const uint8_t MailboxStatusMask = (1<<3);
      if (SyncManStatus & MailboxStatusMask) {
        return true;
      }
    }      
    if (safe_clock_gettime(CLOCK_MONOTONIC, &current_time)!=0) {
      return false;
      }
    timediff = timediff_ms(current_time, start_time);
    safe_usleep(100);
  } while (timediff < MAX_WAIT_TIME_MS);
  
  if (good_results == 0) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " error reading from device\n", __func__);          
  } else {
    fprintf(stderr, "%s : " ERROR_HDR 
            " error read mbx not full after %d ms\n", __func__, timediff);      
  }

  return false;
}


/*!
 * \brief  Waits until write mailbox is empty or timeout.
 *
 * Wait times out after 100msec.
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \return          returns true for success, false for failure or timeout
 */
bool WGMailbox::waitForWriteMailboxReady(EthercatCom *com)
{
  // Wait upto 100ms for device to toggle ack
  static const int MAX_WAIT_TIME_MS = 100;
  int timediff;
  unsigned good_results=0;


  struct timespec start_time, current_time;
  if (safe_clock_gettime(CLOCK_MONOTONIC, &start_time)!=0) {
    return false;
  }
  
  do {      
    // Check if mailbox is full by looking at bit 3 of SyncMan status register.
    uint8_t SyncManStatus=0;
    const unsigned SyncManAddr = 0x805+(MBX_COMMAND_SYNCMAN_NUM*8);
    if (EthercatDevice::readData(com, sh_, SyncManAddr, &SyncManStatus, sizeof(SyncManStatus), EthercatDevice::FIXED_ADDR) == 0) {
      ++good_results;
      const uint8_t MailboxStatusMask = (1<<3);
      if ( !(SyncManStatus & MailboxStatusMask) ) {
        return true;
      }
    }      
    if (safe_clock_gettime(CLOCK_MONOTONIC, &current_time)!=0) {
      return false;
    }
    timediff = timediff_ms(current_time, start_time);
    safe_usleep(100);
  } while (timediff < MAX_WAIT_TIME_MS);
  
  if (good_results == 0) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " error reading from device\n", __func__);          
  } else {
    fprintf(stderr, "%s : " ERROR_HDR 
            " error write mbx not empty after %d ms\n", __func__, timediff);      
  }

  return false;
}



/*!
 * \brief  Writes data to mailbox.
 *
 * Will try to conserve bandwidth by only length bytes of data and last byte of mailbox.
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \param data      pointer to buffer where read data is stored.
 * \param length    amount of data to read from mailbox
 * \return          returns true for success, false for failure
 */
bool WGMailbox::writeMailboxInternal(EthercatCom *com, void const *data, unsigned length)
{
  if (length > MBX_COMMAND_SIZE) {
    assert(length <= MBX_COMMAND_SIZE);
    return false;
  }

  // Make sure slave is in correct state to use mailbox
  if (!verifyDeviceStateForMailboxOperation()){
    return false;
  }

  EC_Logic *logic = EC_Logic::instance();    
  EC_UINT station_addr = sh_->get_station_address();
  

  // If there enough savings, split mailbox write up into 2 parts : 
  //  1. Write of actual data to begining of mbx buffer
  //  2. Write of last mbx buffer byte, to complete write
  static const unsigned TELEGRAM_OVERHEAD = 50;
  bool split_write = (length+TELEGRAM_OVERHEAD) < MBX_COMMAND_SIZE;
    
  unsigned write_length = MBX_COMMAND_SIZE;
  if (split_write) {
    write_length = length;
  }

  // Possible do multiple things at once...
  //  1. Clear read mailbox by reading both first and last mailbox bytes
  //  2. Write data into write mailbox
  {
    // Build frame with 2-NPRD + 2 NPWR
    unsigned char unused[1] = {0};
    NPWR_Telegram write_start(
                              logic->get_idx(),
                              station_addr,
                              MBX_COMMAND_PHY_ADDR,
                              logic->get_wkc(),
                              write_length,
                              (const unsigned char*) data);
    NPWR_Telegram write_end(
                            logic->get_idx(),
                            station_addr,
                            MBX_COMMAND_PHY_ADDR+MBX_COMMAND_SIZE-1,
                            logic->get_wkc(),
                            sizeof(unused),
                            unused);
      
    if (split_write) {
      write_start.attach(&write_end);
    }      

    EC_Ethernet_Frame frame(&write_start);
      
    // Try multiple times, but remember number of of successful sends
    unsigned sends=0;      
    bool success=false;
    for (unsigned tries=0; (tries<10) && !success; ++tries) {
      success = com->txandrx_once(&frame);
      if (!success) {
        updateIndexAndWkc(&write_start, logic);
        updateIndexAndWkc(&write_end, logic);
      }
      ++sends; //EtherCAT_com d/n support split TX and RX class, assume tx part of txandrx always succeeds
      /* 
      int handle = com->tx(&frame);
      if (handle > 0) {
        ++sends;
        success = com->rx(&frame, handle);
      }
      if (!success) {
        updateIndexAndWkc(&write_start, logic);
        updateIndexAndWkc(&write_end, logic);
      }
      */
    }
    if (!success) {
      fprintf(stderr, "%s : " ERROR_HDR 
              " too much packet loss\n", __func__);   
      safe_usleep(100);
      return false;
    }
      
    if (split_write && (write_start.get_wkc() != write_end.get_wkc())) {
      fprintf(stderr, "%s : " ERROR_HDR 
              " write mbx working counters are inconsistant\n", __func__);
      return false;
    }

    if (write_start.get_wkc() > 1) 
    {
      fprintf(stderr, "%s : " ERROR_HDR
              " multiple (%d) devices responded to mailbox write\n", __func__, write_start.get_wkc());
      return false;
    }
    else if (write_start.get_wkc() != 1)
    {
      // Write to cmd mbx was refused 
      if (sends<=1) {
        // Packet was only sent once, there must be a problem with slave device
        fprintf(stderr, "%s : " ERROR_HDR 
                " initial mailbox write refused\n", __func__);
        safe_usleep(100);
        return false;
      } else {
        // Packet was sent multiple times because a packet drop occured  
        // If packet drop occured on return path from device, a refusal is acceptable
        fprintf(stderr, "%s : " WARN_HDR 
                " repeated mailbox write refused\n", __func__);
      }
    }     
  }

  return true;
}

bool WGMailbox::readMailboxRepeatRequest(EthercatCom *com)
{
  bool success = _readMailboxRepeatRequest(com);
  ++mailbox_diagnostics_.retries_;
  if (!success) {
    ++mailbox_diagnostics_.retry_errors_;
  }
  return success;
}

bool WGMailbox::_readMailboxRepeatRequest(EthercatCom *com)
{
  // Toggle repeat request flag, wait for ack from device
  // Returns true if ack is received, false for failure
  SyncMan sm;
  if (!sm.readData(com, sh_, EthercatDevice::FIXED_ADDR, MBX_STATUS_SYNCMAN_NUM)) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " could not read status mailbox syncman (1)\n", __func__);
    return false;
  }
  
  // If device can handle repeat requests, then request and ack bit should already match
  if (sm.activate.repeat_request != sm.pdi_control.repeat_ack) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " syncman repeat request and ack do not match\n", __func__);
    return false;
  }

  // Write toggled repeat request,,, wait for ack.
  SyncManActivate orig_activate(sm.activate);
  sm.activate.repeat_request = ~orig_activate.repeat_request;
  if (!sm.activate.writeData(com, sh_, EthercatDevice::FIXED_ADDR, MBX_STATUS_SYNCMAN_NUM)) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " could not write syncman repeat request\n", __func__);
    //ec_mark(sh->getEM(), "could not write syncman repeat request", 1);
    return false;
  }
  
  // Wait upto 100ms for device to toggle ack
  static const int MAX_WAIT_TIME_MS = 100;
  int timediff;

  struct timespec start_time, current_time;
  if (safe_clock_gettime(CLOCK_MONOTONIC, &start_time)!=0) {
    return false;
  }
  
  do {
    if (!sm.readData(com, sh_, EthercatDevice::FIXED_ADDR, MBX_STATUS_SYNCMAN_NUM)) {
      fprintf(stderr, "%s : " ERROR_HDR 
              " could not read status mailbox syncman (2)\n", __func__);
      return false;
    }

    if (sm.activate.repeat_request == sm.pdi_control.repeat_ack) {
      // Device responded, to some checks to make sure it seems to be telling the truth
      if (sm.status.mailbox_status != 1) {
        fprintf(stderr, "%s : " ERROR_HDR 
                " got repeat response, but read mailbox is still empty\n", __func__);
        //sm.print(WG0X_MBX_Status_Syncman_Num, std::cerr);
        return false;
      }
      return true;
    }
    
    if ( (sm.activate.repeat_request) == (orig_activate.repeat_request) ) {          
      fprintf(stderr, "%s : " ERROR_HDR 
              " syncman repeat request was changed while waiting for response\n", __func__);
      //sm.activate.print();
      //orig_activate.print();
      return false;
    }

    if (safe_clock_gettime(CLOCK_MONOTONIC, &current_time)!=0) {
      return false;
    }
    
    timediff = timediff_ms(current_time, start_time);
    safe_usleep(100);        
  } while (timediff < MAX_WAIT_TIME_MS);
    
  fprintf(stderr, "%s : " ERROR_HDR 
          " error repeat request not acknowledged after %d ms\n", __func__, timediff);    
  return false;
}



/*!
 * \brief  Reads data from read mailbox.
 *
 * Will try to conserve bandwidth by reading length bytes of data and last byte of mailbox.
 * Mailbox lock should be held when this function is called.
 *
 * \param com       used to perform communication with device
 * \param data      pointer to buffer where read data is stored.
 * \param length    amount of data to read from mailbox
 * \return          returns true for success, false for failure
 */
bool WGMailbox::readMailboxInternal(EthercatCom *com, void *data, unsigned length)
{
  static const unsigned MAX_TRIES = 10;
  static const unsigned MAX_DROPPED = 10;
    
  if (length > MBX_STATUS_SIZE) {
    assert(length <= MBX_STATUS_SIZE);
    return false;
  }

  // Make sure slave is in correct state to use mailbox
  if (!verifyDeviceStateForMailboxOperation()){
    return false;
  }
    
  EC_Logic *logic = EC_Logic::instance();    
  EC_UINT station_addr = sh_->get_station_address();


  // If read is small enough :
  //  1. read just length bytes in one telegram
  //  2. then read last byte to empty mailbox
  static const unsigned TELEGRAM_OVERHEAD = 50;
  bool split_read = (length+TELEGRAM_OVERHEAD) < MBX_STATUS_SIZE;
    
  unsigned read_length = MBX_STATUS_SIZE;      
  if (split_read) {
    read_length = length;
 }

  unsigned char unused[1] = {0};
  NPRD_Telegram read_start(
                           logic->get_idx(),
                           station_addr,
                           MBX_STATUS_PHY_ADDR,
                           logic->get_wkc(),
                           read_length,
                           (unsigned char*) data);
  NPRD_Telegram read_end(  
                         logic->get_idx(),
                         station_addr,
                         MBX_STATUS_PHY_ADDR+MBX_STATUS_SIZE-1,
                         logic->get_wkc(),
                         sizeof(unused),
                         unused);      

  if (split_read) {
    read_start.attach(&read_end);
  }
    
  EC_Ethernet_Frame frame(&read_start);

  unsigned tries = 0;    
  unsigned total_dropped =0;
  for (tries=0; tries<MAX_TRIES; ++tries) {      

    // Send read - keep track of how many packets were dropped (for later)
    unsigned dropped=0;
    for (dropped=0; dropped<MAX_DROPPED; ++dropped) {
      if (com->txandrx_once(&frame)) {
        break;
      }
      ++total_dropped;
      updateIndexAndWkc(&read_start   , logic);
      updateIndexAndWkc(&read_end     , logic);
    }
      
    if (dropped>=MAX_DROPPED) {
      fprintf(stderr, "%s : " ERROR_HDR 
              " too many dropped packets : %d\n", __func__, dropped);
    }
      
    if (split_read && (read_start.get_wkc() != read_end.get_wkc())) {
      fprintf(stderr, "%s : " ERROR_HDR 
              "read mbx working counters are inconsistant\n", __func__);
      return false;
    }
      
    if (read_start.get_wkc() == 0) {
      if (dropped == 0) {
        fprintf(stderr, "%s : " ERROR_HDR 
                " inconsistancy : got wkc=%d with no dropped packets\n", 
                __func__, read_start.get_wkc()); 
        fprintf(stderr, "total dropped = %d\n", total_dropped);
        return false;
      } else {
        // Packet was dropped after doing read from device,,,
        // Ask device to repost data, so it can be read again.
        fprintf(stderr, "%s : " WARN_HDR 
                " asking for read repeat after dropping %d packets\n", __func__, dropped);
        if (!readMailboxRepeatRequest(com)) {
          return false;
        }
        continue;
      }
    } else if (read_start.get_wkc() == 1) {
      // Successfull read of status data
      break;
    } else {
      fprintf(stderr, "%s : " ERROR_HDR 
              " invalid wkc for read : %d\n", __func__, read_start.get_wkc());   
      diagnoseMailboxError(com);
      return false;
    }
  }

  if (tries >= MAX_TRIES) {
    fprintf(stderr, "%s : " ERROR_HDR 
            " could not get responce from device after %d retries, %d total dropped packets\n",
            __func__, tries, total_dropped);
    diagnoseMailboxError(com);
    return false;
  }        

  return true;
}


/*!
 * \brief  Read data from WG0X local bus using mailbox communication.
 *
 * Internally a localbus read is done in two parts.
 * First, a mailbox write of a command header that include local bus address and length.
 * Second, a mailbox read of the result.
 *
 * \param com       used to perform communication with device
 * \param address   WG0X (FPGA) local bus address to read from
 * \param data      pointer to buffer where read data can be stored, must be at least length in size
 * \param length    amount of data to read, limited at 511 bytes.
 * \return          returns zero for success, non-zero for failure 
 */
int WGMailbox::readMailbox(EthercatCom *com, unsigned address, void *data, unsigned length)
{
  if (!lockMailbox())
    return -1;

  int result = readMailbox_(com, address, data, length);
  if (result != 0) {
    ++mailbox_diagnostics_.read_errors_;
  }
  
  unlockMailbox();
  return result;
}

/*!
 * \brief  Internal function.  
 *
 * Aguments are the same as readMailbox() except that this assumes the mailbox lock is held.
 */ 
int WGMailbox::readMailbox_(EthercatCom *com, unsigned address, void *data, unsigned length)
{
  // Make sure slave is in correct state to use mailbox
  if (!verifyDeviceStateForMailboxOperation()){
    return false;
  }

  //  1. Clear read (status) mailbox by reading it first
  if (!clearReadMailbox(com)) 
  {
    fprintf(stderr, "%s : " ERROR_HDR 
            " clearing read mbx\n", __func__);
    return -1;
  }

  //  2. Put a (read) request into command mailbox
  {
    WG0XMbxCmd cmd;      
    if (!cmd.build(address, length, LOCAL_BUS_READ, sh_->get_mbx_counter(), data)) 
    {
      fprintf(stderr, "%s : " ERROR_HDR 
              " builing mbx header\n", __func__);
      return -1;
    }
    
    if (!writeMailboxInternal(com, &cmd.hdr_, sizeof(cmd.hdr_))) 
    {
      fprintf(stderr, "%s : " ERROR_HDR " write of cmd failed\n", __func__);
      return -1;
    }
  }
  
  // Wait for result (in read mailbox) to become ready
  if (!waitForReadMailboxReady(com)) 
  {
    fprintf(stderr, "%s : " ERROR_HDR 
            "waiting for read mailbox\n", __func__);
    return -1;
  }

  // Read result back from mailbox.
  // It could take the FPGA some time to respond to a request.  
  // Since the read mailbox is initiall cleared, any read to the mailbox
  // should be refused (WKC==0) until WG0x FPGA has written it result into it.	   
  // NOTE: For this to work the mailbox syncmanagers must be set up.
  // TODO 1: Packets may get lost on return route to device.
  //   In this case, the device will keep responding to the repeated packets with WKC=0.
  //   To work correctly, the repeat request bit needs to be toggled.
  // TODO 2: Need a better method to determine if data read from status mailbox.
  //   is the right data, or just junk left over from last time.
  { 
    WG0XMbxCmd stat;
    memset(&stat,0,sizeof(stat));
    // Read data + 1byte checksum from mailbox
    if (!readMailboxInternal(com, &stat, length+1)) 
    {
      fprintf(stderr, "%s : " ERROR_HDR " read failed\n", __func__);
      return -1;
    }
    
    if (wg_util::computeChecksum(&stat, length+1) != 0) 
    {
      fprintf(stderr, "%s : " ERROR_HDR 
              "checksum error reading mailbox data\n", __func__);
      fprintf(stderr, "length = %d\n", length);
      return -1;
    }
    memcpy(data, &stat, length);
  }

  return 0;


}

bool WGMailbox::lockMailbox() 
{
  int error = pthread_mutex_lock(&mailbox_lock_);
  if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " getting mbx lock\n", __func__);
    ++mailbox_diagnostics_.lock_errors_;
    return false;
  }
  return true;
}

void WGMailbox::unlockMailbox() 
{
  int error = pthread_mutex_unlock(&mailbox_lock_);
  if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " freeing mbx lock\n", __func__);
    ++mailbox_diagnostics_.lock_errors_;
  }
}


/*!
 * \brief  Write data to WG0X local bus using mailbox communication.
 *
 * First, this puts a command header that include local bus address and length in write mailbox.
 * Second it waits until device actually empties write mailbox.
 *
 * \param com       used to perform communication with device
 * \param address   WG0X (FPGA) local bus address to write data to
 * \param data      pointer to buffer where write data is stored, must be at least length in size
 * \param length    amount of data to write, limited at 507 bytes
 * \return          returns zero for success, non-zero for failure 
 */
int WGMailbox::writeMailbox(EthercatCom *com, unsigned address, void const *data, unsigned length)
{
  if (!lockMailbox())
    return -1;

  int result = writeMailbox_(com, address, data, length);
  if (result != 0) {
    ++mailbox_diagnostics_.write_errors_;
  }

  unlockMailbox();

  return result;
}

/*!
 * \brief  Internal function.  
 *
 * Aguments are the same as writeMailbox() except that this assumes the mailbox lock is held.
 */
int WGMailbox::writeMailbox_(EthercatCom *com, unsigned address, void const *data, unsigned length)
{
  // Make sure slave is in correct state to use mailbox
  if (!verifyDeviceStateForMailboxOperation()){
    return -1;
  }
    
  // Build message and put it into write mailbox
  {		
    WG0XMbxCmd cmd;
    if (!cmd.build(address, length, LOCAL_BUS_WRITE, sh_->get_mbx_counter(), data)) {
      fprintf(stderr, "%s : " ERROR_HDR " builing mbx header\n", __func__);
      return -1;
    }      
    
    unsigned write_length = sizeof(cmd.hdr_)+length+sizeof(cmd.checksum_);
    if (!writeMailboxInternal(com, &cmd, write_length)) {
      fprintf(stderr, "%s : " ERROR_HDR " write failed\n", __func__);
      diagnoseMailboxError(com);
      return -1;
    }
  }
  
  // TODO: Change slave firmware so that we can verify that localbus write was truly executed
  //  Checking that device emptied write mailbox will have to suffice for now.
  if (!waitForWriteMailboxReady(com)) {
    fprintf(stderr, "%s : " ERROR_HDR 
            "write mailbox\n", __func__);
  }
    
  return 0;
}


void WGMailbox::publishMailboxDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
{
  if (lockMailbox()) { 
    mailbox_publish_diagnostics_ = mailbox_diagnostics_;
    unlockMailbox();
  }

  MbxDiagnostics const &m(mailbox_publish_diagnostics_);
  d.addf("Mailbox Write Errors", "%d", m.write_errors_);
  d.addf("Mailbox Read Errors", "%d",  m.read_errors_);
  d.addf("Mailbox Retries", "%d",      m.retries_);
  d.addf("Mailbox Retry Errors", "%d", m.retry_errors_);
}


}; //end namespace ethercat_hardware
