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

#include <iomanip>

#include <math.h>
#include <stddef.h>

#include <ethercat_hardware/wg0x.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <boost/crc.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/static_assert.hpp>
#include <boost/make_shared.hpp>

// Temporary,, need 'log' fuction that can switch between fprintf and ROS_LOG.
#define ERR_MODE "\033[41m"
#define STD_MODE "\033[0m"
#define WARN_MODE "\033[43m"
#define GOOD_MODE "\033[42m"
#define INFO_MODE "\033[44m"

#define ERROR_HDR "\033[41mERROR\033[0m"
#define WARN_HDR "\033[43mERROR\033[0m"

#include "ethercat_hardware/wg_util.h"


WG0XDiagnostics::WG0XDiagnostics() :
  first_(true),
  valid_(false),
  safety_disable_total_(0),
  undervoltage_total_(0),
  over_current_total_(0),
  board_over_temp_total_(0),
  bridge_over_temp_total_(0),
  operate_disable_total_(0),
  watchdog_disable_total_(0),
  lock_errors_(0),
  checksum_errors_(0),
  zero_offset_(0),
  cached_zero_offset_(0)
{
  memset(&safety_disable_status_, 0, sizeof(safety_disable_status_));
  memset(&diagnostics_info_, 0, sizeof(diagnostics_info_));
}

/*!
 * \brief  Use new updates WG0X diagnostics with new safety disable data
 *
 * \param new_status    newly collected safety disable status 
 * \param new_counters  newly collected safety disable counters
 */
void WG0XDiagnostics::update(const WG0XSafetyDisableStatus &new_status, const WG0XDiagnosticsInfo &new_diagnostics_info)
{
  first_ = false;
  safety_disable_total_   += 0xFF & ((uint32_t)(new_status.safety_disable_count_ - safety_disable_status_.safety_disable_count_));
  {
    const WG0XSafetyDisableCounters &new_counters(new_diagnostics_info.safety_disable_counters_);
    const WG0XSafetyDisableCounters &old_counters(diagnostics_info_.safety_disable_counters_);
    undervoltage_total_     += 0xFF & ((uint32_t)(new_counters.undervoltage_count_ - old_counters.undervoltage_count_));
    over_current_total_     += 0xFF & ((uint32_t)(new_counters.over_current_count_ - old_counters.over_current_count_));
    board_over_temp_total_  += 0xFF & ((uint32_t)(new_counters.board_over_temp_count_ - old_counters.board_over_temp_count_));
    bridge_over_temp_total_ += 0xFF & ((uint32_t)(new_counters.bridge_over_temp_count_ - old_counters.bridge_over_temp_count_));
    operate_disable_total_  += 0xFF & ((uint32_t)(new_counters.operate_disable_count_ - old_counters.operate_disable_count_));
    watchdog_disable_total_ += 0xFF & ((uint32_t)(new_counters.watchdog_disable_count_ - old_counters.watchdog_disable_count_));
  } 

  safety_disable_status_   = new_status;
  diagnostics_info_        = new_diagnostics_info;
}


/*!
 * \brief  Verify CRC stored in actuator info structure 
 *
 * ActuatorInfo now constains two CRCs.
 * Originally all devices had EEPROMS with 264 byte pages, and only crc264 was used.  
 * However, support was need for EEPROM with 246 byte pages.
 * To have backwards compatible support, there is also a CRC of first 252 (256-4) bytes.
 *
 * Devices configure in past will only have 264 byte EEPROM pages, and 264byte CRC.  
 * Newer devices might have 256 or 264 byte pages.  
 * The 264 byte EEPROMs will store both CRCs.  
 * The 256 byte EEPROMs will only store the 256 byte CRC.
 * 
 * Thus:
 *  - Old software will be able to use 264 byte EEPROM with new dual CRC.
 *  - New software will be able to use 264 byte EEPROM with single 264 byte CRC
 *  - Only new sofware will be able to use 256 byte EEPROM
 *
 * \param com       EtherCAT communication class used for communicating with device
 * \return          true if CRC is good, false if CRC is invalid
 */
bool WG0XActuatorInfo::verifyCRC() const
{
  // Actuator info contains two CRCs
  BOOST_STATIC_ASSERT(sizeof(WG0XActuatorInfo) == 264);
  BOOST_STATIC_ASSERT( offsetof(WG0XActuatorInfo, crc32_256_) == (256-4));
  BOOST_STATIC_ASSERT( offsetof(WG0XActuatorInfo, crc32_264_) == (264-4));
  boost::crc_32_type crc32_256, crc32_264;  
  crc32_256.process_bytes(this, offsetof(WG0XActuatorInfo, crc32_256_));
  crc32_264.process_bytes(this, offsetof(WG0XActuatorInfo, crc32_264_));
  return ((this->crc32_264_ == crc32_264.checksum()) || (this->crc32_256_ == crc32_256.checksum()));
}

/*!
 * \brief  Calculate CRC of structure and update crc32_256_ and crc32_264_ elements
 */
void WG0XActuatorInfo::generateCRC(void)
{
  boost::crc_32_type crc32;
  crc32.process_bytes(this, offsetof(WG0XActuatorInfo, crc32_256_));
  this->crc32_256_ = crc32.checksum();
  crc32.reset();
  crc32.process_bytes(this, offsetof(WG0XActuatorInfo, crc32_264_));
  this->crc32_264_ = crc32.checksum();
}


WG0X::WG0X() :
  max_current_(0.0),
  too_many_dropped_packets_(false),
  status_checksum_error_(false),
  timestamp_jump_detected_(false),
  fpga_internal_reset_detected_(false),
  encoder_errors_detected_(false),
  cached_zero_offset_(0), 
  calibration_status_(NO_CALIBRATION),
  app_ram_status_(APP_RAM_MISSING),
  motor_model_(NULL),
  disable_motor_model_checking_(false)
{

  last_timestamp_ = 0;
  last_last_timestamp_ = 0;
  drops_ = 0;
  consecutive_drops_ = 0;
  max_consecutive_drops_ = 0;
  max_board_temperature_ = 0;
  max_bridge_temperature_ = 0;
  in_lockout_ = false;
  resetting_ = false;
  has_error_ = false;

  int error;
  if ((error = pthread_mutex_init(&wg0x_diagnostics_lock_, NULL)) != 0)
  {
    ROS_ERROR("WG0X : init diagnostics mutex :%s", strerror(error));
  }

}

WG0X::~WG0X()
{
  delete sh_->get_fmmu_config();
  delete sh_->get_pd_config();
  delete motor_model_;
}


void WG0X::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  EthercatDevice::construct(sh, start_address);

  // WG EtherCAT devices (WG05,WG06,WG21) revisioning scheme
  fw_major_ = (sh->get_revision() >> 8) & 0xff;
  fw_minor_ = sh->get_revision() & 0xff;
  board_major_ = ((sh->get_revision() >> 24) & 0xff) - 1;
  board_minor_ = (sh->get_revision() >> 16) & 0xff;

  // Would normally configure EtherCAT initialize EtherCAT communication settings here.
  // However, since all WG devices are slightly different doesn't make sense to do it here.
  // Instead make sub-classes handle this.
}


/**  \brief Fills in ethercat_hardware::ActuatorInfo from WG0XActuatorInfo
 *
 * WG0XAcuatorInfo is a packed structure that comes directly from the device EEPROM.
 * ethercat_hardware::ActuatorInfo is a ROS message type that is used by both
 * motor model and motor heating model. 
 */
void WG0X::copyActuatorInfo(ethercat_hardware::ActuatorInfo &out,  const WG0XActuatorInfo &in)
{
  out.id   = in.id_;
  out.name = std::string(in.name_);
  out.robot_name = in.robot_name_;
  out.motor_make = in.motor_make_;
  out.motor_model = in.motor_model_;
  out.max_current = in.max_current_;
  out.speed_constant = in.speed_constant_; 
  out.motor_resistance  = in.resistance_;
  out.motor_torque_constant = in.motor_torque_constant_;
  out.encoder_reduction = in.encoder_reduction_;
  out.pulses_per_revolution = in.pulses_per_revolution_;  
}


/**  \brief Allocates and initialized motor trace for WG0X devices than use it (WG006, WG005)
 */
bool WG0X::initializeMotorModel(pr2_hardware_interface::HardwareInterface *hw, 
                                const string &device_description,
                                double max_pwm_ratio, 
                                double board_resistance,
                                bool   poor_measured_motor_voltage)
{
  if (!hw) 
    return true;

  motor_model_ = new MotorModel(1000);
  if (motor_model_ == NULL) 
    return false;

  const ethercat_hardware::ActuatorInfo &ai(actuator_info_msg_);
  
  unsigned product_code = sh_->get_product_code();
  ethercat_hardware::BoardInfo bi;
  bi.description = device_description; 
  bi.product_code = product_code;
  bi.pcb = board_major_;
  bi.pca = board_minor_;
  bi.serial = sh_->get_serial();
  bi.firmware_major = fw_major_;
  bi.firmware_minor = fw_minor_;
  bi.board_resistance = board_resistance;
  bi.max_pwm_ratio    = max_pwm_ratio;
  bi.hw_max_current   = config_info_.absolute_current_limit_ * config_info_.nominal_current_scale_;
  bi.poor_measured_motor_voltage = poor_measured_motor_voltage;

  if (!motor_model_->initialize(ai,bi))
    return false;
  
  // Create digital out that can be used to force trigger of motor trace
  publish_motor_trace_.name_ = string(actuator_info_.name_) + "_publish_motor_trace";
  publish_motor_trace_.command_.data_ = 0;
  publish_motor_trace_.state_.data_ = 0;
  if (!hw->addDigitalOut(&publish_motor_trace_)) {
    ROS_FATAL("A digital out of the name '%s' already exists", publish_motor_trace_.name_.c_str());
    return false;
  }

  // When working with experimental setups we don't want motor model to halt motors when it detects a problem.
  // Allow rosparam to disable motor model halting for a specific motor.
  if (!ros::param::get("~/" + ai.name + "/disable_motor_model_checking", disable_motor_model_checking_))
  {
    disable_motor_model_checking_ = false;
  }
  if (disable_motor_model_checking_)
  {
    ROS_WARN("Disabling motor model on %s", ai.name.c_str());
  }

  return true;
}


boost::shared_ptr<ethercat_hardware::MotorHeatingModelCommon> WG0X::motor_heating_model_common_;

bool WG0X::initializeMotorHeatingModel(bool allow_unprogrammed)
{

  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
  ethercat_hardware::MotorHeatingModelParametersEepromConfig config;
  if (!readMotorHeatingModelParametersFromEeprom(&com, config))
  {
    ROS_FATAL("Unable to read motor heating model config parameters from EEPROM");
    return false;
  }

  // All devices need to have motor model heating model parameters stored in them...
  // Even if device doesn't use paramers, they should be there.
  if (!config.verifyCRC())
  {
    if (allow_unprogrammed)
    {
      ROS_WARN("%s EEPROM does not contain motor heating model parameters",
               actuator_info_.name_);
      return true;
    }
    else 
    {
      ROS_WARN("%s EEPROM does not contain motor heating model parameters",
               actuator_info_.name_);
      return true;
      // TODO: once there is ability to update all MCB iwth motorconf, this is will become a fatal error
      ROS_FATAL("%s EEPROM does not contain motor heating model parameters", 
                actuator_info_.name_);
      return false;
    }
  }

  // Even though all devices should contain motor heating model parameters,
  // The heating model does not need to be used.
  if (config.enforce_ == 0)
  {
    return true;
  }

  // Don't need motor model if we are not using ROS (motorconf)
  if (!use_ros_)
  {
    return true;
  }

  // Generate hwid for motor model
  std::ostringstream hwid;
  hwid << unsigned(sh_->get_product_code()) << std::setw(5) << std::setfill('0') << unsigned(sh_->get_serial());

  // All motor heating models use shared settings structure
  if (motor_heating_model_common_.get() == NULL)
  {
    ros::NodeHandle nh("~motor_heating_model");
    motor_heating_model_common_ = boost::make_shared<ethercat_hardware::MotorHeatingModelCommon>(nh);
    motor_heating_model_common_->initialize();
    // Only display following warnings once.
    if (!motor_heating_model_common_->enable_model_)
    {
      ROS_WARN("Motor heating model disabled for all devices");
    }
    if (!motor_heating_model_common_->load_save_files_)
    {
      ROS_WARN("Not loading motor heating model files");
    }
    if (!motor_heating_model_common_->update_save_files_)
    {
      ROS_WARN("Not saving motor heating model files");
    }
  }

  if (!motor_heating_model_common_->enable_model_)
  {
    return true;
  }
    
  motor_heating_model_ = 
    boost::make_shared<ethercat_hardware::MotorHeatingModel>(config.params_, 
                                                             actuator_info_.name_, 
                                                             hwid.str(),
                                                             motor_heating_model_common_->save_directory_); 
  // have motor heating model load last saved temperaures from filesystem
  if (motor_heating_model_common_->load_save_files_)
  {
    if (!motor_heating_model_->loadTemperatureState())
    {
      ROS_WARN("Could not load motor temperature state for %s", actuator_info_.name_);
    }
  }
  if (motor_heating_model_common_->publish_temperature_)
  {
    motor_heating_model_->startTemperaturePublisher();
  }
  motor_heating_model_common_->attach(motor_heating_model_);

  return true;
}


int WG0X::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  ROS_DEBUG("Device #%02d: WG0%d (%#08x) Firmware Revision %d.%02d, PCB Revision %c.%02d, Serial #: %d", 
            sh_->get_ring_position(),
            sh_->get_product_code() % 100,
            sh_->get_product_code(), fw_major_, fw_minor_,
            'A' + board_major_, board_minor_,
            sh_->get_serial());

  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());

  mailbox_.initialize(sh_);

  if (sh_->get_product_code() == WG05_PRODUCT_CODE)
  {
    if (fw_major_ != 1 || fw_minor_ < 7)
    {
      ROS_FATAL("Unsupported firmware revision %d.%02d", fw_major_, fw_minor_);
      return -1;
    }
  }
  else
  {
    if ((fw_major_ == 0 && fw_minor_ < 4) /*|| (fw_major_ == 1 && fw_minor_ < 0)*/)
    {
      ROS_FATAL("Unsupported firmware revision %d.%02d", fw_major_, fw_minor_);
      return -1;
    }
  }

  if (readMailbox(&com, WG0XConfigInfo::CONFIG_INFO_BASE_ADDR, &config_info_, sizeof(config_info_)) != 0)
  {
    ROS_FATAL("Unable to load configuration information");
    return -1;
  }
  ROS_DEBUG("            Serial #: %05d", config_info_.device_serial_number_);
  double board_max_current = double(config_info_.absolute_current_limit_) * config_info_.nominal_current_scale_;

  if (!readActuatorInfoFromEeprom(&com, actuator_info_))
  {
    ROS_FATAL("Unable to read actuator info from EEPROM");
    return -1;
  }
  
  if (actuator_info_.verifyCRC())
  {
    if (actuator_info_.major_ != 0 || actuator_info_.minor_ != 2)
    {
      if (allow_unprogrammed)
        ROS_WARN("Unsupported actuator info version (%d.%d != 0.2).  Please reprogram device #%02d", actuator_info_.major_, actuator_info_.minor_, sh_->get_ring_position());
      else
      {
        ROS_FATAL("Unsupported actuator info version (%d.%d != 0.2).  Please reprogram device #%02d", actuator_info_.major_, actuator_info_.minor_, sh_->get_ring_position());
        return -1;
      }
    }

    actuator_.name_ = actuator_info_.name_;
    ROS_DEBUG("            Name: %s", actuator_info_.name_);

    // Copy actuator info read from eeprom, into msg type
    copyActuatorInfo(actuator_info_msg_, actuator_info_);

    if (!initializeMotorHeatingModel(allow_unprogrammed))
    {
      return -1;
    }


    bool isWG021 = sh_->get_product_code() == WG021_PRODUCT_CODE;
    if (!isWG021)
    {
      // Register actuator with pr2_hardware_interface::HardwareInterface
      if (hw && !hw->addActuator(&actuator_))
      {
          ROS_FATAL("An actuator of the name '%s' already exists.  Device #%02d has a duplicate name", actuator_.name_.c_str(), sh_->get_ring_position());
          return -1;
      }

    }

    // Register digital out with pr2_hardware_interface::HardwareInterface
    digital_out_.name_ = actuator_info_.name_;
    if (hw && !hw->addDigitalOut(&digital_out_))
    {
        ROS_FATAL("A digital out of the name '%s' already exists.  Device #%02d has a duplicate name", digital_out_.name_.c_str(), sh_->get_ring_position());
        return -1;
    }

    // If it is supported, read application ram data.
    if (app_ram_status_ == APP_RAM_PRESENT)
    {
      double zero_offset;
      if (readAppRam(&com, zero_offset))
      {
        ROS_DEBUG("Read calibration from device %s: %f", actuator_info_.name_, zero_offset);
        actuator_.state_.zero_offset_ = zero_offset;
        cached_zero_offset_ = zero_offset;
        calibration_status_ = SAVED_CALIBRATION;
      }
      else
      {
        ROS_DEBUG("No calibration offset was stored on device %s", actuator_info_.name_);
      }
    }
    else if (app_ram_status_ == APP_RAM_MISSING)
    {
      ROS_WARN("Device %s does not support storing calibration offsets", actuator_info_.name_);
    }
    else if (app_ram_status_ == APP_RAM_NOT_APPLICABLE)
    {
      // don't produce warning
    }

    // Make sure motor current limit is less than board current limit
    if (actuator_info_.max_current_ > board_max_current)
    {
      ROS_WARN("WARNING: Device #%02d : motor current limit (%f) greater than board current limit (%f)", 
               sh_->get_ring_position(), actuator_info_.max_current_, board_max_current);
    }
    max_current_ = std::min(board_max_current, actuator_info_.max_current_);
  }
  else if (allow_unprogrammed)
  {
    ROS_WARN("WARNING: Device #%02d (%d%05d) is not programmed", 
             sh_->get_ring_position(), sh_->get_product_code(), sh_->get_serial());
    //actuator_info_.crc32_264_ = 0;
    //actuator_info_.crc32_256_ = 0;

    max_current_ = board_max_current;
  }
  else
  {
    ROS_FATAL("Device #%02d (%d%05d) is not programmed, aborting...", 
              sh_->get_ring_position(), sh_->get_product_code(), sh_->get_serial());
    return -1;
  }

  return 0;
}

#define GET_ATTR(a) \
{ \
  TiXmlElement *c; \
  attr = elt->Attribute((a)); \
  if (!attr) { \
    c = elt->FirstChildElement((a)); \
    if (!c || !(attr = c->GetText())) { \
      ROS_FATAL("Actuator is missing the attribute "#a); \
      exit(EXIT_FAILURE); \
    } \
  } \
}

void WG0X::clearErrorFlags(void)
{
  has_error_ = false;
  too_many_dropped_packets_ = false;
  status_checksum_error_ = false;
  timestamp_jump_detected_ = false;
  encoder_errors_detected_ = false;
  if (motor_model_) motor_model_->reset();
  if (motor_heating_model_.get() != NULL) 
  {
    motor_heating_model_->reset();
  }
}

void WG0X::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  pr2_hardware_interface::ActuatorCommand &cmd = actuator_.command_;
  
  if (halt) 
  {
    cmd.effort_ = 0;
  }

  if (reset) 
  {
    clearErrorFlags();
  }
  resetting_ = reset;

  // If zero_offset was changed, give it to non-realtime thread
  double zero_offset = actuator_.state_.zero_offset_;
  if (zero_offset != cached_zero_offset_) 
  {
    if (tryLockWG0XDiagnostics())
    {
      ROS_DEBUG("Calibration change of %s, new %f, old %f", actuator_info_.name_, zero_offset, cached_zero_offset_);
      cached_zero_offset_ = zero_offset;
      wg0x_collect_diagnostics_.zero_offset_ = zero_offset;
      calibration_status_ = CONTROLLER_CALIBRATION;
      unlockWG0XDiagnostics();
    }
    else 
    {
      // It is OK if trylock failed, this will still try again next cycle.
    }
  }

  // Compute the current
  double current = (cmd.effort_ / actuator_info_.encoder_reduction_) / actuator_info_.motor_torque_constant_ ;
  actuator_.state_.last_commanded_effort_ = cmd.effort_;
  actuator_.state_.last_commanded_current_ = current;

  // Truncate the current to limit
  current = max(min(current, max_current_), -max_current_);

  // Pack command structures into EtherCAT buffer
  WG0XCommand *c = (WG0XCommand *)buffer;
  memset(c, 0, command_size_);
  c->programmed_current_ = int(current / config_info_.nominal_current_scale_);
  c->mode_ = (cmd.enable_ && !halt && !has_error_) ? (MODE_ENABLE | MODE_CURRENT) : MODE_OFF;
  c->mode_ |= (reset ? MODE_SAFETY_RESET : 0);
  c->digital_out_ = digital_out_.command_.data_;
  c->checksum_ = wg_util::rotateRight8(wg_util::computeChecksum(c, command_size_ - 1));
}

bool WG0X::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  pr2_hardware_interface::ActuatorState &state = actuator_.state_;
  WG0XStatus *this_status, *prev_status;

  this_status = (WG0XStatus *)(this_buffer + command_size_);
  prev_status = (WG0XStatus *)(prev_buffer + command_size_);

  digital_out_.state_.data_ = this_status->digital_out_;

  // Do not report timestamp directly to controllers because 32bit integer 
  // value in microseconds will overflow every 72 minutes.   
  // Instead a accumulate small time differences into a ros::Duration variable
  int32_t timediff = WG0X::timestampDiff(this_status->timestamp_, prev_status->timestamp_);
  sample_timestamp_ += WG0X::timediffToDuration(timediff);
  state.sample_timestamp_ = sample_timestamp_;   //ros::Duration is preferred source of time for controllers
  state.timestamp_ = sample_timestamp_.toSec();  //double value is for backwards compatibility
  
  state.device_id_ = sh_->get_ring_position();
  
  state.encoder_count_ = this_status->encoder_count_;
  state.position_ = double(this_status->encoder_count_) / actuator_info_.pulses_per_revolution_ * 2 * M_PI - state.zero_offset_;
  
  state.encoder_velocity_ = 
    calcEncoderVelocity(this_status->encoder_count_, this_status->timestamp_,
                        prev_status->encoder_count_, prev_status->timestamp_);
  state.velocity_ = state.encoder_velocity_ / actuator_info_.pulses_per_revolution_ * 2 * M_PI;

  state.calibration_reading_ = this_status->calibration_reading_ & LIMIT_SENSOR_0_STATE;
  state.calibration_rising_edge_valid_ = this_status->calibration_reading_ &  LIMIT_OFF_TO_ON;
  state.calibration_falling_edge_valid_ = this_status->calibration_reading_ &  LIMIT_ON_TO_OFF;
  state.last_calibration_rising_edge_ = double(this_status->last_calibration_rising_edge_) / actuator_info_.pulses_per_revolution_ * 2 * M_PI;
  state.last_calibration_falling_edge_ = double(this_status->last_calibration_falling_edge_) / actuator_info_.pulses_per_revolution_ * 2 * M_PI;
  state.is_enabled_ = bool(this_status->mode_ & MODE_ENABLE);

  state.last_executed_current_ = this_status->programmed_current_ * config_info_.nominal_current_scale_;
  state.last_measured_current_ = this_status->measured_current_ * config_info_.nominal_current_scale_;

  state.last_executed_effort_ = this_status->programmed_current_ * config_info_.nominal_current_scale_ * actuator_info_.motor_torque_constant_ * actuator_info_.encoder_reduction_;
  state.last_measured_effort_ = this_status->measured_current_ * config_info_.nominal_current_scale_ * actuator_info_.motor_torque_constant_ * actuator_info_.encoder_reduction_;

  state.num_encoder_errors_ = this_status->num_encoder_errors_;

  state.motor_voltage_ = this_status->motor_voltage_ * config_info_.nominal_voltage_scale_;

  state.max_effort_ = max_current_ * actuator_info_.encoder_reduction_ * actuator_info_.motor_torque_constant_; 

  return verifyState(this_status, prev_status);
}


bool WG0X::verifyChecksum(const void* buffer, unsigned size)
{
  bool success = wg_util::computeChecksum(buffer, size) == 0;
  if (!success) {
    if (tryLockWG0XDiagnostics()) {
      ++wg0x_collect_diagnostics_.checksum_errors_;
      unlockWG0XDiagnostics();
    }
  }
  return success;
}



/**
 * Returns (new_timestamp - old_timestamp).  Accounts for wrapping of timestamp values.
 *
 * It is assumed that each timestamps is exactly 32bit and can wrap around from 0xFFFFFFFF back to 0. 
 * In this case  1 - 4294967295 should equal 2 not -4294967294.  (Note : 0xFFFFFFFF = 4294967295)
 */
int32_t WG0X::timestampDiff(uint32_t new_timestamp, uint32_t old_timestamp)
{
  return int32_t(new_timestamp - old_timestamp);
}

/**
 * Convert timestamp difference to ros::Duration.  Timestamp is assumed to be in microseconds
 *
 * It is assumed that each timestamps is exactly 32bit and can wrap around from 0xFFFFFFFF back to 0. 
 * In this case  1 - 4294967295 should equal 2 not -4294967294.  (Note : 0xFFFFFFFF = 4294967295)
 */
ros::Duration WG0X::timediffToDuration(int32_t timediff_usec)
{
  static const int USEC_PER_SEC = 1000000;
  int sec  = timediff_usec / USEC_PER_SEC;
  int nsec = (timediff_usec % USEC_PER_SEC)*1000;
  return ros::Duration(sec,nsec);
}


/**
 * Returns (new_position - old_position).  Accounts for wrap-around of 32-bit position values.
 *
 * It is assumed that each position value is exactly 32bit and can wrap from -2147483648 to +2147483647.
 */
int32_t WG0X::positionDiff(int32_t new_position, int32_t old_position)
{
  return int32_t(new_position - old_position);
}

/**
 * Returns velocity in encoder ticks per second.
 *
 * Timestamp assumed to be in microseconds
 * Accounts for wrapping of timestamp values and position values.
 */
double WG0X::calcEncoderVelocity(int32_t new_position, uint32_t new_timestamp, 
                                 int32_t old_position, uint32_t old_timestamp)
{
  double timestamp_diff = double(timestampDiff(new_timestamp, old_timestamp)) * 1e-6;
  double position_diff = double(positionDiff(new_position, old_position));
  return (position_diff / timestamp_diff);
}


/**
 * \brief  Converts raw 16bit temperature value returned by device into value in degress Celcius
 * 
 * \param raw_temp  Raw 16bit temperature value return by device
 * \return          Temperature in degrees Celcius
 */
double WG0X::convertRawTemperature(int16_t raw_temp)
{
  return 0.0078125 * double(raw_temp);
}


// Returns true if timestamp changed by more than (amount) or time goes in reverse.
bool WG0X::timestamp_jump(uint32_t timestamp, uint32_t last_timestamp, uint32_t amount)
{
  uint32_t timestamp_diff = (timestamp - last_timestamp);
  return (timestamp_diff > amount);
}

bool WG0X::verifyState(WG0XStatus *this_status, WG0XStatus *prev_status)
{
  pr2_hardware_interface::ActuatorState &state = actuator_.state_;
  bool rv = true;

  if ((motor_model_ != NULL) || (motor_heating_model_ != NULL))
  {
    // Both motor model and motor heating model use MotorTraceSample
    ethercat_hardware::MotorTraceSample &s(motor_trace_sample_);
    double last_executed_current =  this_status->programmed_current_ * config_info_.nominal_current_scale_;
    double supply_voltage = double(prev_status->supply_voltage_) * config_info_.nominal_voltage_scale_;
    double pwm_ratio = double(this_status->programmed_pwm_value_) / double(PWM_MAX);
    s.timestamp        = state.timestamp_;
    s.enabled          = state.is_enabled_;
    s.supply_voltage   = supply_voltage;
    s.measured_motor_voltage = state.motor_voltage_;
    s.programmed_pwm   = pwm_ratio;
    s.executed_current = last_executed_current;
    s.measured_current = state.last_measured_current_;
    s.velocity         = state.velocity_;
    s.encoder_position = state.position_;
    s.encoder_error_count = state.num_encoder_errors_;

    if (motor_model_ != NULL)
    {
      // Collect data for motor model
      motor_model_->sample(s);
      motor_model_->checkPublish();
    }
    if (motor_heating_model_ != NULL)
    {
      double ambient_temperature = convertRawTemperature(this_status->board_temperature_);
      double duration = double(timestampDiff(this_status->timestamp_, prev_status->timestamp_)) * 1e-6;
      motor_heating_model_->update(s, actuator_info_msg_, ambient_temperature, duration);

      if ((!motor_heating_model_common_->disable_halt_) && (motor_heating_model_->hasOverheated()))
      {
        rv = false;
      }
    }
  }

  max_board_temperature_ = max(max_board_temperature_, this_status->board_temperature_);
  max_bridge_temperature_ = max(max_bridge_temperature_, this_status->bridge_temperature_);

  if (this_status->timestamp_ == last_timestamp_ ||
      this_status->timestamp_ == last_last_timestamp_) {
    ++drops_;
    ++consecutive_drops_;
    max_consecutive_drops_ = max(max_consecutive_drops_, consecutive_drops_);
  } else {
    consecutive_drops_ = 0;
  }
  // Detect timestamps going in reverse or changing by more than 10 seconds = 10,000,000 usec
  if ( timestamp_jump(this_status->timestamp_,last_timestamp_,10000000) )
  {
    timestamp_jump_detected_ = true;
  }
  last_last_timestamp_ = last_timestamp_;
  last_timestamp_ = this_status->timestamp_;

  if (consecutive_drops_ > 10)
  {
    too_many_dropped_packets_ = true;
    rv = false;
    goto end;
  }

  in_lockout_ = bool(this_status->mode_ & MODE_SAFETY_LOCKOUT);
  if (in_lockout_ && !resetting_)
  {
    rv = false;
    goto end;
  }

  if (fpga_internal_reset_detected_)
  {
    rv = false;
    goto end;
  }

  if (this_status->num_encoder_errors_ != prev_status->num_encoder_errors_)
  {
    encoder_errors_detected_ = true;
  }

  if (state.is_enabled_ && motor_model_)
  {
    if (!disable_motor_model_checking_)
    {
      if(!motor_model_->verify())
      {
        // Motor model will automatically publish a motor trace when there is an error
        rv = false;
        goto end;
      }
    }
  }

end:
  if (motor_model_) 
  {
    // Publish trace when:
    //  * device goes into safety lockout
    //  * controller request motor trace to be published
    bool new_error = in_lockout_ && !resetting_ && !has_error_;
    if (new_error || publish_motor_trace_.command_.data_)
    {
      const char* reason = "Publishing manually triggered";
      if (new_error)
      {
        bool undervoltage = (this_status->mode_ & MODE_UNDERVOLTAGE);    
        reason = (undervoltage) ? "Undervoltage Lockout" : "Safety Lockout";
      }    
      int level          = (new_error) ? 2 : 0;
      motor_model_->flagPublish(reason, level , 100);
      publish_motor_trace_.command_.data_ = 0;
    }
  }
  bool is_error = !rv;
  has_error_ = is_error || has_error_;
  actuator_.state_.halted_ = has_error_ || this_status->mode_ == MODE_OFF;
  return rv;
}

bool WG0X::publishTrace(const string &reason, unsigned level, unsigned delay)
{
  if (motor_model_) 
  {
    motor_model_->flagPublish(reason, level, delay);
    return true;
  }
  return false;
}


void WG0X::collectDiagnostics(EthercatCom *com)
{
  //Collect safety disable information through mailbox  
  bool success = false;

  // Have parent collect diagnositcs
  EthercatDevice::collectDiagnostics(com);

  // Send a packet with both a Fixed address read (NPRW) to device to make sure it is present in chain.
  // This avoids wasting time trying to read mailbox of device that it not present on chain.
  {
    EC_Logic *logic = EC_Logic::instance();
    unsigned char buf[1];
    EC_UINT address = 0x0000;
    NPRD_Telegram nprd_telegram(logic->get_idx(),
                                sh_->get_station_address(),
                                address,
                                0 /*working counter*/,
                                sizeof(buf),
                                buf);
    EC_Ethernet_Frame frame(&nprd_telegram);
    if (!com->txandrx_once(&frame)) {
      // packet didn't come back
      goto end;
    }
    if (nprd_telegram.get_wkc() != 1) {
      // packet came back, but device didn't not respond
      goto end;
    }
  }
 
  WG0XSafetyDisableStatus s;
  if (readMailbox(com, s.BASE_ADDR, &s, sizeof(s)) != 0) {
    goto end;
  }
    
  WG0XDiagnosticsInfo di;
  if (readMailbox(com, di.BASE_ADDR, &di, sizeof(di)) != 0) {
    goto end;
  }
  
  { // Try writing zero offset to to WG0X devices that have application ram
    WG0XDiagnostics &dg(wg0x_collect_diagnostics_);

    if ((app_ram_status_ == APP_RAM_PRESENT) && (dg.zero_offset_ != dg.cached_zero_offset_))
    {
      if (writeAppRam(com, dg.zero_offset_)){
	ROS_DEBUG("Writing new calibration to device %s, new %f, old %f", actuator_info_.name_, dg.zero_offset_, dg.cached_zero_offset_);
	dg.cached_zero_offset_ = dg.zero_offset_;
      }
      else{
	ROS_ERROR("Failed to write new calibration to device %s, new %f, old %f", actuator_info_.name_, dg.zero_offset_, dg.cached_zero_offset_);
	// Diagnostics thread will try again next update cycle
      }
    }
  }

  success = true;

 end:
  if (!lockWG0XDiagnostics()) {
    wg0x_collect_diagnostics_.valid_ = false;   // change these values even if we did't get the lock
    wg0x_collect_diagnostics_.first_ = false;   
    return;
  }

  wg0x_collect_diagnostics_.valid_ = success;   
  if (success) {
    wg0x_collect_diagnostics_.update(s,di);
  }

  unlockWG0XDiagnostics();
}


bool WG0X::writeAppRam(EthercatCom *com, double zero_offset) 
{
  WG0XUserConfigRam cfg;
  cfg.version_ = 1;
  cfg.zero_offset_ = zero_offset;
  boost::crc_32_type crc32;
  crc32.process_bytes(&cfg, sizeof(cfg)-sizeof(cfg.crc32_));
  cfg.crc32_ = crc32.checksum();
  return (writeMailbox(com, WG0XUserConfigRam::BASE_ADDR, &cfg, sizeof(cfg)) == 0);
}

bool WG0X::readAppRam(EthercatCom *com, double &zero_offset) 
{
  WG0XUserConfigRam cfg;
  if (!readMailbox(com, WG0XUserConfigRam::BASE_ADDR, &cfg, sizeof(cfg)) == 0)
  {
    return false;
  }
  if (cfg.version_ != 1) 
  {
    return false;
  }
  boost::crc_32_type crc32;
  crc32.process_bytes(&cfg, sizeof(cfg)-sizeof(cfg.crc32_));
  if (cfg.crc32_ != crc32.checksum()) {
    return false;
  }
  zero_offset = cfg.zero_offset_;
  return true;
}


/*!
 * \brief  Reads actuator info from eeprom.  
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \param acuator_info Structure where actuator info will be stored.
 * \return          true if there is success, false if there is an error
 */
bool WG0X::readActuatorInfoFromEeprom(EthercatCom *com, WG0XActuatorInfo &actuator_info)
{
  BOOST_STATIC_ASSERT(sizeof(actuator_info) == 264);

  if (!eeprom_.readEepromPage(com, &mailbox_, ACTUATOR_INFO_PAGE, &actuator_info, sizeof(actuator_info)))
  {
    ROS_ERROR("Reading acutuator info from eeprom");
    return false;
  }
  return true;
}
 
/*!
 * \brief  Reads actuator info from eeprom.  
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \param acuator_info Structure where actuator info will be stored.
 * \return          true if there is success, false if there is an error
 */
bool WG0X::readMotorHeatingModelParametersFromEeprom(EthercatCom *com, MotorHeatingModelParametersEepromConfig &config)
{
  BOOST_STATIC_ASSERT(sizeof(config) == 256);

  if (!eeprom_.readEepromPage(com, &mailbox_, config.EEPROM_PAGE, &config, sizeof(config)))
  {
    ROS_ERROR("Reading motor heating model config from eeprom");
    return false;
  }
  return true;
}




/*!
 * \brief  Programs acutator and heating parameters into device EEPROM.
 *
 * WG0X devices store configuaration info in EEPROM.  This configuration information contains
 * information such as device name, motor parameters, and encoder parameters.  
 *
 * Originally, devices only stored ActuatorInfo in EEPROM.
 * However, later we discovered that in extreme cases, certain motors may overheat.  
 * To prevent motor overheating, a model is used to estimate motor winding temperature
 * and stop motor if temperature gets too high.  
 * However, the new motor heating model needs more motor parameters than were originally stored
 * in eeprom. 
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \param actutor_info  Actuator information to be stored in device EEPROM
 * \param actutor_info  Motor heating motor information to be stored in device EEPROM
 * \return          true if there is success, false if there is an error
 */
bool WG0X::program(EthercatCom *com, const WG0XActuatorInfo &actutor_info)
{
  if (!eeprom_.writeEepromPage(com, &mailbox_, ACTUATOR_INFO_PAGE, &actutor_info, sizeof(actutor_info)))
  {
    ROS_ERROR("Writing actuator infomation to EEPROM");
    return false;
  }
  
  return true;
}


/*!
 * \brief  Programs motor heating parameters into device EEPROM.
 *
 * Originally, devices only stored ActuatorInfo in EEPROM.
 * However, later we discovered that in extreme cases, certain motors may overheat.  
 * To prevent motor overheating, a model is used to estimate motor winding temperature
 * and stop motor if temperature gets too high.  
 * However, the new motor heating model needs more motor parameters than were originally stored
 * in eeprom. 
 * 
 * \param com       EtherCAT communication class used for communicating with device
 * \param heating_config  Motor heating model parameters to be stored in device EEPROM
 * \return          true if there is success, false if there is an error
 */
bool WG0X::program(EthercatCom *com, const ethercat_hardware::MotorHeatingModelParametersEepromConfig &heating_config)
{
  if (!eeprom_.writeEepromPage(com, &mailbox_, heating_config.EEPROM_PAGE, &heating_config, sizeof(heating_config)))
  {
    ROS_ERROR("Writing motor heating model configuration to EEPROM");
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
int WG0X::readMailbox(EthercatCom *com, unsigned address, void *data, unsigned length)
{
  return mailbox_.readMailbox(com, address, data, length);
}

bool WG0X::lockWG0XDiagnostics() 
{
  int error = pthread_mutex_lock(&wg0x_diagnostics_lock_);
  if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " getting diagnostics lock\n", __func__);
    // update error counters even if we didn't get lock
    ++wg0x_collect_diagnostics_.lock_errors_;
    return false;
  }
  return true;
}

bool WG0X::tryLockWG0XDiagnostics() 
{
  int error = pthread_mutex_trylock(&wg0x_diagnostics_lock_);
  if (error == EBUSY) {
    return false;
  }
  else if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " getting diagnostics lock\n", __func__);
    // update error counters even if we didn't get lock
    ++wg0x_collect_diagnostics_.lock_errors_;
    return false;
  }
  return true;
}

void WG0X::unlockWG0XDiagnostics() 
{
  int error = pthread_mutex_unlock(&wg0x_diagnostics_lock_);
  if (error != 0) {
    fprintf(stderr, "%s : " ERROR_HDR " freeing diagnostics lock\n", __func__);
    ++wg0x_collect_diagnostics_.lock_errors_;
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
int WG0X::writeMailbox(EthercatCom *com, unsigned address, void const *data, unsigned length)
{
  return mailbox_.writeMailbox(com,address,data,length);
}



#define CHECK_SAFETY_BIT(bit) \
  do { if (status & SAFETY_##bit) { \
    str += prefix + #bit; \
    prefix = ", "; \
  } } while(0)

string WG0X::safetyDisableString(uint8_t status)
{
  string str, prefix;

  if (status & SAFETY_DISABLED)
  {
    CHECK_SAFETY_BIT(DISABLED);
    CHECK_SAFETY_BIT(UNDERVOLTAGE);
    CHECK_SAFETY_BIT(OVER_CURRENT);
    CHECK_SAFETY_BIT(BOARD_OVER_TEMP);
    CHECK_SAFETY_BIT(HBRIDGE_OVER_TEMP);
    CHECK_SAFETY_BIT(OPERATIONAL);
    CHECK_SAFETY_BIT(WATCHDOG);
  }
  else
    str = "ENABLED";

  return str;
}

string WG0X::modeString(uint8_t mode)
{
  string str, prefix;
  if (mode) {
    if (mode & MODE_ENABLE) {
      str += prefix + "ENABLE";
      prefix = ", ";
    }
    if (mode & MODE_CURRENT) {
      str += prefix + "CURRENT";
      prefix = ", ";
    }
    if (mode & MODE_UNDERVOLTAGE) {
      str += prefix + "UNDERVOLTAGE";
      prefix = ", ";
    }
    if (mode & MODE_SAFETY_RESET) {
      str += prefix + "SAFETY_RESET";
      prefix = ", ";
    }
    if (mode & MODE_SAFETY_LOCKOUT) {
      str += prefix + "SAFETY_LOCKOUT";
      prefix = ", ";
    }
    if (mode & MODE_RESET) {
      str += prefix + "RESET";
      prefix = ", ";
    }
  } else {
    str = "OFF";
  }
  return str;
}

void WG0X::publishGeneralDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
{ 
  // If possible, copy new diagnositics from collection thread, into diagnostics thread
  if (tryLockWG0XDiagnostics()) { 
    wg0x_publish_diagnostics_ = wg0x_collect_diagnostics_;
    unlockWG0XDiagnostics(); 
  }

  if (too_many_dropped_packets_)
  {
    d.mergeSummary(d.ERROR, "Too many dropped packets");
  }

  if (status_checksum_error_)
  {
    d.mergeSummary(d.ERROR, "Checksum error on status data");
  }
  
  if (wg0x_publish_diagnostics_.first_)
  {
    d.mergeSummary(d.WARN, "Have not yet collected WG0X diagnostics");
  }
  else if (!wg0x_publish_diagnostics_.valid_) 
  {
    d.mergeSummary(d.WARN, "Could not collect WG0X diagnostics");
  }

  WG0XDiagnostics const &p(wg0x_publish_diagnostics_);
  WG0XSafetyDisableStatus const &s(p.safety_disable_status_);
  d.addf("Status Checksum Error Count", "%d", p.checksum_errors_);
  d.addf("Safety Disable Status", "%s (%02x)", safetyDisableString(s.safety_disable_status_).c_str(), s.safety_disable_status_);
  d.addf("Safety Disable Status Hold", "%s (%02x)", safetyDisableString(s.safety_disable_status_hold_).c_str(), s.safety_disable_status_hold_);
  d.addf("Safety Disable Count", "%d", p.safety_disable_total_);
  d.addf("Undervoltage Count", "%d", p.undervoltage_total_);
  d.addf("Over Current Count", "%d", p.over_current_total_);
  d.addf("Board Over Temp Count", "%d", p.board_over_temp_total_);
  d.addf("Bridge Over Temp Count", "%d", p.bridge_over_temp_total_);
  d.addf("Operate Disable Count", "%d", p.operate_disable_total_);
  d.addf("Watchdog Disable Count", "%d", p.watchdog_disable_total_);

  if (in_lockout_)
  {
    uint8_t status = s.safety_disable_status_hold_;
    string prefix(": "); 
    string str("Safety Lockout");
    CHECK_SAFETY_BIT(UNDERVOLTAGE);
    CHECK_SAFETY_BIT(OVER_CURRENT);
    CHECK_SAFETY_BIT(BOARD_OVER_TEMP);
    CHECK_SAFETY_BIT(HBRIDGE_OVER_TEMP);
    CHECK_SAFETY_BIT(OPERATIONAL);
    CHECK_SAFETY_BIT(WATCHDOG);
    d.mergeSummary(d.ERROR, str);
  }

  if (timestamp_jump_detected_ && (s.safety_disable_status_hold_ & SAFETY_OPERATIONAL))
  {
    fpga_internal_reset_detected_ = true;
  }

  if (fpga_internal_reset_detected_) 
  {
    d.mergeSummaryf(d.ERROR, "FPGA internal reset detected");
  }
  
  if (timestamp_jump_detected_)
  {
    d.mergeSummaryf(d.WARN, "Timestamp jumped");
  }

  {

    const WG0XDiagnosticsInfo &di(p.diagnostics_info_);
    //d.addf("PDO Command IRQ Count", "%d", di.pdo_command_irq_count_);
    d.addf("MBX Command IRQ Count", "%d", di.mbx_command_irq_count_);
    d.addf("PDI Timeout Error Count", "%d", di.pdi_timeout_error_count_);
    d.addf("PDI Checksum Error Count", "%d", di.pdi_checksum_error_count_);
    unsigned product = sh_->get_product_code();

    // Current scale 
    if ((product == WG05_PRODUCT_CODE) && (board_major_ == 1))
    {
      // WG005B measure current going into and out-of H-bridge (not entire board)
      static const double WG005B_SUPPLY_CURRENT_SCALE = (1.0 / (8152.0 * 0.851)) * 4.0;
      double bridge_supply_current = double(di.supply_current_in_) * WG005B_SUPPLY_CURRENT_SCALE;
      d.addf("Bridge Supply Current", "%f", bridge_supply_current);
    }
    else if ((product == WG05_PRODUCT_CODE) || (product == WG021_PRODUCT_CODE)) 
    {
      // WG005[CDEF] measures curret going into entire board.  It cannot measure negative (regenerative) current values.
      // WG021A == WG005E,  WG021B == WG005F
      static const double WG005_SUPPLY_CURRENT_SCALE = ((82.0 * 2.5) / (0.01 * 5100.0 * 32768.0));
      double supply_current = double(di.supply_current_in_) * WG005_SUPPLY_CURRENT_SCALE;
      d.addf("Supply Current", "%f",  supply_current);
    }
    d.addf("Configured Offset A", "%f", config_info_.nominal_current_scale_ * di.config_offset_current_A_);
    d.addf("Configured Offset B", "%f", config_info_.nominal_current_scale_ * di.config_offset_current_B_);
  }
}



void WG0X::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  WG0XStatus *status = (WG0XStatus *)(buffer + command_size_);

  stringstream str;
  str << "EtherCAT Device (" << actuator_info_.name_ << ")";
  d.name = str.str();
  char serial[32];
  snprintf(serial, sizeof(serial), "%d-%05d-%05d", config_info_.product_id_ / 100000 , config_info_.product_id_ % 100000, config_info_.device_serial_number_);
  d.hardware_id = serial;

  d.summary(d.OK, "OK");

  d.clear();
  d.add("Configuration", config_info_.configuration_status_ ? "good" : "error loading configuration");
  d.add("Name", actuator_info_.name_);
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product code",
        "WG0%d (%d) Firmware Revision %d.%02d, PCB Revision %c.%02d",
        sh_->get_product_code() == WG05_PRODUCT_CODE ? 5 : 6,
        sh_->get_product_code(), fw_major_, fw_minor_,
        'A' + board_major_, board_minor_);

  d.add("Robot", actuator_info_.robot_name_);
  d.addf("Motor", "%s %s", actuator_info_.motor_make_, actuator_info_.motor_model_);
  d.add("Serial Number", serial);
  d.addf("Nominal Current Scale", "%f",  config_info_.nominal_current_scale_);
  d.addf("Nominal Voltage Scale",  "%f", config_info_.nominal_voltage_scale_);
  d.addf("HW Max Current", "%f", config_info_.absolute_current_limit_ * config_info_.nominal_current_scale_);

  d.addf("SW Max Current", "%f", actuator_info_.max_current_);
  d.addf("Speed Constant", "%f", actuator_info_.speed_constant_);
  d.addf("Resistance", "%f", actuator_info_.resistance_);
  d.addf("Motor Torque Constant", "%f", actuator_info_.motor_torque_constant_);
  d.addf("Pulses Per Revolution", "%d", actuator_info_.pulses_per_revolution_);
  d.addf("Encoder Reduction", "%f", actuator_info_.encoder_reduction_);

  publishGeneralDiagnostics(d);
  mailbox_.publishMailboxDiagnostics(d);

  d.addf("Calibration Offset", "%f", cached_zero_offset_);
  d.addf("Calibration Status", "%s", 
         (calibration_status_ == NO_CALIBRATION) ? "No calibration" :
         (calibration_status_ == CONTROLLER_CALIBRATION) ? "Calibrated by controller" :
         (calibration_status_ == SAVED_CALIBRATION) ? "Using saved calibration" : "UNKNOWN");

  d.addf("Watchdog Limit", "%dms", config_info_.watchdog_limit_);
  d.add("Mode", modeString(status->mode_));
  d.addf("Digital out", "%d", status->digital_out_);
  d.addf("Programmed pwm value", "%d", status->programmed_pwm_value_);
  d.addf("Programmed current", "%f", status->programmed_current_ * config_info_.nominal_current_scale_);
  d.addf("Measured current", "%f", status->measured_current_ * config_info_.nominal_current_scale_);
  d.addf("Timestamp", "%u", status->timestamp_);
  d.addf("Encoder count", "%d", status->encoder_count_);
  d.addf("Encoder index pos", "%d", status->encoder_index_pos_);
  d.addf("Num encoder_errors", "%d", status->num_encoder_errors_);
  d.addf("Encoder status", "%d", status->encoder_status_);
  d.addf("Calibration reading", "%d", status->calibration_reading_);
  d.addf("Last calibration rising edge", "%d", status->last_calibration_rising_edge_);
  d.addf("Last calibration falling edge", "%d", status->last_calibration_falling_edge_);
  d.addf("Board temperature", "%f", 0.0078125 * status->board_temperature_);
  d.addf("Max board temperature", "%f", 0.0078125 * max_board_temperature_);
  d.addf("Bridge temperature", "%f", 0.0078125 * status->bridge_temperature_);
  d.addf("Max bridge temperature", "%f", 0.0078125 * max_bridge_temperature_);
  d.addf("Supply voltage", "%f", status->supply_voltage_ * config_info_.nominal_voltage_scale_);
  d.addf("Motor voltage", "%f", status->motor_voltage_ * config_info_.nominal_voltage_scale_);
  d.addf("Current Loop Kp", "%d", config_info_.current_loop_kp_);
  d.addf("Current Loop Ki", "%d", config_info_.current_loop_ki_);

  if (motor_model_) 
  {
    motor_model_->diagnostics(d);
    if (disable_motor_model_checking_)
    {
      d.mergeSummaryf(d.WARN, "Motor model disabled");      
    }
  }

  if (motor_heating_model_.get() != NULL)
  {
    motor_heating_model_->diagnostics(d);
  }

  if (encoder_errors_detected_)
  {
    d.mergeSummaryf(d.WARN, "Encoder errors detected");
  }

  d.addf("Packet count", "%d", status->packet_count_);

  d.addf("Drops", "%d", drops_);
  d.addf("Consecutive Drops", "%d", consecutive_drops_);
  d.addf("Max Consecutive Drops", "%d", max_consecutive_drops_);

  unsigned numPorts = (sh_->get_product_code()==WG06_PRODUCT_CODE) ? 1 : 2; // WG006 has 1 port, WG005 has 2
  EthercatDevice::ethercatDiagnostics(d, numPorts); 
}

