#include <ethercat_hardware/wg_soft_processor.h>

#include <sstream>
#include <boost/foreach.hpp>
#include <boost/static_assert.hpp>

namespace ethercat_hardware
{

WGSoftProcessor::WGSoftProcessor()
{

}

bool WGSoftProcessor::initialize(EthercatCom *com)
{
  com_ = com;
  ros::NodeHandle nh("~/soft_processor/");
  read_firmware_service_ = nh.advertiseService("read_firmware", &WGSoftProcessor::readFirmwareCB, this);
  write_firmware_service_ = nh.advertiseService("write_firmware", &WGSoftProcessor::writeFirmwareCB, this);
  reset_service_ = nh.advertiseService("reset", &WGSoftProcessor::resetCB, this);
  return true;
}


void WGSoftProcessor::add(WGMailbox *mbx,
                          const std::string &actuator_name, 
                          const std::string &processor_name,
                          unsigned iram_address, unsigned ctrl_address)
{
  Info info(mbx, actuator_name, processor_name, iram_address, ctrl_address);
  processors_.push_back(info);
  ROS_INFO("Processor : %s/%s", actuator_name.c_str(), processor_name.c_str());
}



const WGSoftProcessor::Info* WGSoftProcessor::get(const std::string &actuator_name, 
                                                  const std::string &processor_name,
                                                  std::ostream &err_out) const
{
  BOOST_FOREACH(const Info &info, processors_)
  {
    if ((info.actuator_name_ == actuator_name) && (info.processor_name_ == processor_name))
    {
      return &info;
    }
  }

  err_out << "No actuator/processor with name " << actuator_name << "/" << processor_name;
  return NULL;
}



bool WGSoftProcessor::readFirmwareCB(ethercat_hardware::SoftProcessorFirmwareRead::Request &request, 
                                     ethercat_hardware::SoftProcessorFirmwareRead::Response &response)
{
  response.success = false;
  response.error_msg = "";

  std::ostringstream err_out;

  const Info *info = get(request.actuator_name, request.processor_name, err_out);
  if (!info)
  {
    response.error_msg = err_out.str();
    return true;
  }

  // Each instruction is maped to 32bit memory array.  Read 64 instructions at a time.
  response.instructions.resize(IRAM_INSTRUCTION_LENGTH);
  static const unsigned INSTRUCTION_READ_CHUNK = 64;
  uint8_t buf[INSTRUCTION_READ_CHUNK*4]; // Each instruction in 4 bytes
  
  BOOST_STATIC_ASSERT((IRAM_INSTRUCTION_LENGTH%INSTRUCTION_READ_CHUNK) == 0);

  for (unsigned ii=0; ii<(IRAM_INSTRUCTION_LENGTH/INSTRUCTION_READ_CHUNK); ii+=INSTRUCTION_READ_CHUNK)
  {
    if (info->mbx_->readMailbox(com_, info->iram_address_ + ii*4, buf, sizeof(buf)))
    {
      response.error_msg = "Error reading IRAM data with mailbox";
      return true;
    }
    
    // Data 4-bytes and make integer out of them
    for (unsigned jj=0; jj<INSTRUCTION_READ_CHUNK; ++jj)
    {     
      // instrutions are little endian
      uint32_t instruction = 
        (uint32_t(buf[jj*4+3])<<24) | 
        (uint32_t(buf[jj*4+2])<<16) | 
        (uint32_t(buf[jj*4+1])<<8 ) | 
        (uint32_t(buf[jj*4+0])<<0 ) ;
      
      response.instructions[ii+jj] = instruction;
    }
  }

  response.success = true;
  return true;
}



bool WGSoftProcessor::writeFirmwareCB(ethercat_hardware::SoftProcessorFirmwareWrite::Request &request, 
                                      ethercat_hardware::SoftProcessorFirmwareWrite::Response &response)
{
  response.success = false;
  response.error_msg = "";

  std::ostringstream err_out;

  const Info *info = get(request.actuator_name, request.processor_name, err_out);
  if (!info)
  {
    response.error_msg = err_out.str();
    return true;
  }

  // Put soft-processor in reset before starting to re-write firmware
  if (!assertReset(*info, err_out))
  {
    response.error_msg = err_out.str();    
    return true;
  }

  // perform write here


  // Take soft-processor out of reset now that firmware is completely re-written
  if (!releaseReset(*info, err_out))
  {
    response.error_msg = err_out.str();    
    return true;
  }

  response.success = true;
  return true;
}


bool WGSoftProcessor::resetCB(ethercat_hardware::SoftProcessorReset::Request &request, 
                              ethercat_hardware::SoftProcessorReset::Response &response)
{
  response.success = false;
  response.error_msg = "";

  std::ostringstream err_out;

  const Info *info = get(request.actuator_name, request.processor_name, err_out);
  if (!info)
  {
    response.error_msg = err_out.str();
    return true;
  }

  if (!assertReset(*info, err_out))
  {
    response.error_msg = err_out.str();    
    return true;
  }

  if (!releaseReset(*info, err_out))
  {
    response.error_msg = err_out.str();    
    return true;
  }

  response.success = true;
  return true;  
}


//! Puts soft processor in reset
bool WGSoftProcessor::assertReset(const Info &info, std::ostream &err_msg)
{
  // use mailbox to set bit 0 of byte at info->ctrl_address

  return true;
}

//! Takes soft processor out of reset
bool WGSoftProcessor::releaseReset(const Info &info, std::ostream &err_msg)
{
  // use mailbox to clear bit 0 of byte at info->ctrl_address
  
  return true;
}




}; // end namespace ethercat_hardware

