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

#include "ethercat_hardware/wg_eeprom.h"
#include "ros/ros.h"

#include <boost/static_assert.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

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




WGEeprom::WGEeprom()
{
  
}


/*!
 * \brief  Waits for SPI eeprom state machine to be idle.
 *
 * Polls busy SPI bit of SPI state machine. 
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \return          true if state machine is free, false if there is an error, or we timed out waiting
 */
bool WGEeprom::waitForSpiEepromReady(EthercatCom *com, WGMailbox *mbx)
{
  WG0XSpiEepromCmd cmd;
  // TODO : poll for a given number of millseconds instead of a given number of cycles
  //start_time = 0;
  unsigned tries = 0;
  do {
    //read_time = time;
    ++tries;
    if (!readSpiEepromCmd(com, mbx, cmd))
    {
      ROS_ERROR("Error reading SPI Eeprom Cmd busy bit");
      return false;
    }

    if (!cmd.busy_) 
    {
      return true;
    }       
    
    usleep(100);
  } while (tries <= 10);

  ROS_ERROR("Timed out waiting for SPI state machine to be idle (%d)", tries);
  return false;
}


/*!
 * \brief  Sends command to SPI EEPROM state machine.   
 *
 * This function makes sure SPI EEPROM state machine is idle before sending new command.
 * It also waits for state machine to be idle before returning.
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \return          true if command was send, false if there is an error
 */
bool WGEeprom::sendSpiEepromCmd(EthercatCom *com, WGMailbox *mbx, const WG0XSpiEepromCmd &cmd)
{
  if (!waitForSpiEepromReady(com, mbx))
  {
    return false;
  }

  // Send command
  if (mbx->writeMailbox(com, WG0XSpiEepromCmd::SPI_COMMAND_ADDR, &cmd, sizeof(cmd)))
  {
    ROS_ERROR("Error writing SPI EEPROM command");
    return false;
  }

  // Now read back SPI EEPROM state machine register, and check : 
  //  1. for state machine to become ready
  //  2. that command data was properly write and not corrupted
  WG0XSpiEepromCmd stat;
  unsigned tries = 0;
  do
  {
    if (!readSpiEepromCmd(com, mbx, stat))
    {
      return false;
    }

    if (stat.operation_ != cmd.operation_)
    {
      ROS_ERROR("Invalid readback of SPI EEPROM operation : got 0x%X, expected 0x%X\n", stat.operation_, cmd.operation_);
      return false;
    }

    // return true if command has completed
    if (!stat.busy_)
    {
      if (tries > 0) 
      {
        ROS_WARN("Eeprom state machine took %d cycles", tries);
      }
      return true;;
    }

    fprintf(stderr, "eeprom busy reading again, waiting...\n");
    usleep(100);
  } while (++tries < 10);

  ROS_ERROR("Eeprom SPI state machine busy after %d cycles", tries);
  return false;
}


/*!
 * \brief  Read data from single eeprom page. 
 *
 * Data should be less than 264 bytes.  Note that some eeproms only support 256 byte pages.  
 * If 264 bytes of data are read from a 256 byte eeprom, then last 8 bytes of data will be zeros.
 * Function may block for extended periods of time (is not realtime safe).
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \param mbx       Mailbox for used communication with device
 * \param page      EEPROM page number to read from.  Should be 0 to 4095.
 * \param data      pointer to data buffer
 * \param length    length of data in buffer
 * \return          true if there is success, false if there is an error
 */
bool WGEeprom::readEepromPage(EthercatCom *com, WGMailbox *mbx, unsigned page, void* data, unsigned length)
{
  boost::lock_guard<boost::mutex> lock(mutex_);

  if (length > MAX_EEPROM_PAGE_SIZE)
  {
    ROS_ERROR("Eeprom read length %d > %d", length, MAX_EEPROM_PAGE_SIZE);
    return false;
  }

  if (page >= NUM_EEPROM_PAGES)
  {
    ROS_ERROR("Eeprom read page %d > %d", page, NUM_EEPROM_PAGES-1);
    return false;
  }

  // Since we don't know the size of the eeprom there is not always 264 bytes available.
  // This may try to read 264 bytes, but only the first 256 bytes may be valid.  
  // To avoid any odd issue, zero out FPGA buffer before asking for eeprom data.
  memset(data,0,length);  
  if (mbx->writeMailbox(com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, data, length)) 
  {
    ROS_ERROR("Error zeroing eeprom data buffer");
    return false;
  }

  // Send command to SPI state machine to perform read of eeprom, 
  // sendSpiEepromCmd will automatically wait for SPI state machine 
  // to be idle before a new command is sent
  WG0XSpiEepromCmd cmd;
  memset(&cmd,0,sizeof(cmd));
  cmd.build_read(page);
  if (!sendSpiEepromCmd(com, mbx, cmd)) 
  {
    ROS_ERROR("Error sending SPI read command");
    return false;
  }

  // Wait for SPI Eeprom Read to complete
  // sendSPICommand will wait for Command to finish before returning

  // Read eeprom page data from FPGA buffer
  if (mbx->readMailbox(com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, data, length)) 
  {
    ROS_ERROR("Error reading eeprom data from buffer");
    return false;
  }

  return true;
}


/*!
 * \brief  Write data to single eeprom page. 
 *
 * Data should be less than 264 bytes.  If data size is less than 264 bytes, then 
 * the page will be padded with 0xFF.  Note that some eeproms only support 256 byte
 * pages.  With 256 byte eeproms, the eeprom FW with ingore last 8 bytes of requested write.
 * Function may block for extended periods of time (is not realtime safe).
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \param page      EEPROM page number to write to.  Should be 0 to 4095.
 * \param data      pointer to data buffer
 * \param length    length of data in buffer.  If length < 264, eeprom page will be padded out to 264 bytes.
 * \return          true if there is success, false if there is an error
 */
bool WGEeprom::writeEepromPage(EthercatCom *com, WGMailbox *mbx, unsigned page, const void* data, unsigned length)
{
  boost::lock_guard<boost::mutex> lock(mutex_);

  if (length > 264)
  {
    ROS_ERROR("Eeprom write length %d > %d", length, MAX_EEPROM_PAGE_SIZE);
    return false;
  }

  if (page >= NUM_EEPROM_PAGES)
  {
    ROS_ERROR("Eeprom write page %d > %d", page, NUM_EEPROM_PAGES-1);
    return false;
  }

  // wait for eeprom to be ready before write data into FPGA buffer
  if (!waitForSpiEepromReady(com, mbx))
  {
    return false;
  }

  const void *write_buf = data;

  // if needed, pad data to 264 byte in buf
  uint8_t buf[MAX_EEPROM_PAGE_SIZE];
  if (length < MAX_EEPROM_PAGE_SIZE)
  {
    memcpy(buf, data, length);
    memset(buf+length, 0xFF, MAX_EEPROM_PAGE_SIZE-length);
    write_buf = buf;    
  }

  // Write data to FPGA buffer
  if (mbx->writeMailbox(com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, write_buf, MAX_EEPROM_PAGE_SIZE))
  {
    ROS_ERROR("Write of SPI EEPROM buffer failed");
    return false;
  }

  // Have SPI EEPROM state machine start SPI data transfer
  WG0XSpiEepromCmd cmd;
  cmd.build_write(page);
  if (!sendSpiEepromCmd(com, mbx, cmd)) 
  {
    ROS_ERROR("Error giving SPI EEPROM write command");
    return false;
  }

  // Wait for EEPROM write to complete
  if (!waitForEepromReady(com, mbx))
  {
    return false;
  }

  return true;
}


/*!
 * \brief  Waits for EEPROM to become ready
 *
 * Certain eeprom operations (such as page reads), are complete immediately after data is 
 * trasferred.  Other operations (such as page writes) take some amount of time after data
 * is trasfered to complete.  This polls the EEPROM status register until the 'ready' bit 
 * is set.
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \return          true if there is success, false if there is an error or wait takes too long
 */
bool WGEeprom::waitForEepromReady(EthercatCom *com, WGMailbox *mbx)
{
  // Wait for eeprom write to complete
  unsigned tries = 0;
  EepromStatusReg status_reg;
  do {
    if (!readEepromStatusReg(com, mbx, status_reg))
    {
      return false;
    }
    if (status_reg.ready_)
    {
      break;
    }
    usleep(100);
  } while (++tries < 20);

  if (!status_reg.ready_) 
  {
    ROS_ERROR("Eeprom still busy after %d cycles", tries);
    return false;
  } 

  if (tries > 10)
  {
    ROS_WARN("EEPROM took %d cycles to be ready", tries);
  }
  return true;
}



/*!
 * \brief  Reads EEPROM status register
 *
 * Amoung other things, eeprom status register provide information about whether eeprom 
 * is busy performing a write.
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \param reg       reference to EepromStatusReg struct where eeprom status will be stored
 * \return          true if there is success, false if there is an error
 */
bool WGEeprom::readEepromStatusReg(EthercatCom *com, WGMailbox *mbx, EepromStatusReg &reg)
{
  // Status is read from EEPROM by having SPI state machine perform an "abitrary" operation.
  // With an arbitrary operation, the SPI state machine shifts out byte from buffer, while
  // storing byte shifted in from device into same location in buffer.
  // SPI state machine has no idea what command it is sending device or how to intpret its result.

  // To read eeprom status register, we transfer 2 bytes.  The first byte is the read status register 
  // command value (0xD7).  When transfering the second byte, the EEPROM should send its status.
  unsigned char data[2] = {0xD7, 0x00};
  BOOST_STATIC_ASSERT(sizeof(data) == 2);
  if (mbx->writeMailbox(com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, data, sizeof(data)))
  {
    ROS_ERROR("Writing SPI buffer");
    return false;
  }
    
  { // Have SPI state machine trasfer 2 bytes
    WG0XSpiEepromCmd cmd;
    cmd.build_arbitrary(sizeof(data));
    if (!sendSpiEepromCmd(com, mbx, cmd)) 
    {
      ROS_ERROR("Sending SPI abitrary command");
      return false;
    }
  }
    
  // Data read from device should now be stored in FPGA buffer
  if (mbx->readMailbox(com, WG0XSpiEepromCmd::SPI_BUFFER_ADDR, data, sizeof(data)))
  {
    ROS_ERROR("Reading status register data from SPI buffer");
    return false;
  }
 
  // Status register would be second byte of buffer
  reg.raw_ = data[1];
  return true;
}


/*!
 * \brief  Reads SPI state machine command register
 *
 * For communicating with EEPROM, there is a simple state machine that transfers
 * data to/from FPGA buffer over SPI.  
 * When any type of comunication is done with EEPROM:
 *  1. Write command or write data into FPGA buffer.
 *  2. Have state machine start transfer bytes from buffer to EEPROM, and write data from EEPROM into buffer
 *  3. Wait for state machine to complete (by reading its status)
 *  4. Read EEPROM response from FPGA buffer.
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \param reg       reference to WG0XSpiEepromCmd struct where read data will be stored
 * \return          true if there is success, false if there is an error
 */
bool WGEeprom::readSpiEepromCmd(EthercatCom *com, WGMailbox *mbx, WG0XSpiEepromCmd &cmd)
{
  BOOST_STATIC_ASSERT(sizeof(WG0XSpiEepromCmd) == 3);
  if (mbx->readMailbox(com, WG0XSpiEepromCmd::SPI_COMMAND_ADDR, &cmd, sizeof(cmd)))
  {
    ROS_ERROR("Reading SPI command register with mailbox");
    return false;
  }
  
  return true;
}


}; //end namespace ethercat_hardware
