#include <ethercat_hardware/wg_soft_processor.h>

#include <sstream>

namespace ethercat_hardware
{

WGSoftProcessor::WGSoftProcessor()
{

}

bool WGSoftProcessor::initialize(ros::NodeHandle nh)
{
  ros::NodeHandle nh2("soft_processor");
  read_firmware_service_ = nh2.advertiseService("read_firmware", &WGSoftProcessor::readFirmwareCB, this);
  write_firmware_service_ = nh2.advertiseService("write_firmware", &WGSoftProcessor::writeFirmwareCB, this);
  reset_service_ = nh2.advertiseService("reset", &WGSoftProcessor::resetCB, this);
  return true;
}


void WGSoftProcessor::add(const std::string &name, unsigned iram_address, unsigned ctrl_address)
{
  processors_.insert(ProcessorMap::value_type(name, Info(name, iram_address, ctrl_address)));
}


const WGSoftProcessor::Info* WGSoftProcessor::get(const std::string &name, std::ostream &err_out) const
{
  ProcessorMap::const_iterator iter = processors_.find(name);
  if (iter == processors_.end())
  {
    err_out << "No processor with name : " << name;
    return NULL;
  }
  return &(iter->second);
}



bool WGSoftProcessor::readFirmwareCB(ethercat_hardware::SoftProcessorFirmwareRead::Request &request, 
                                     ethercat_hardware::SoftProcessorFirmwareRead::Response &response)
{
  response.success = false;
  response.error_msg = "";

  std::ostringstream err_out;

  const Info *info = get(request.processor_name, err_out);
  if (!info)
  {
    response.error_msg = err_out.str();
    return true;
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

  const Info *info = get(request.processor_name, err_out);
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

  const Info *info = get(request.processor_name, err_out);
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

