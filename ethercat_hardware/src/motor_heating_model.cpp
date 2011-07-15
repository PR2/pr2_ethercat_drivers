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

#include "ethercat_hardware/motor_heating_model.h"

#include <boost/crc.hpp>
#include <boost/static_assert.hpp>
#include <boost/filesystem.hpp>

// Use XML format when saving or loading XML file
#include <tinyxml.h>

#include <stdio.h>
#include <errno.h>
#include <exception>

namespace ethercat_hardware
{


bool MotorHeatingModelParametersEepromConfig::verifyCRC(void) const
{
  BOOST_STATIC_ASSERT(sizeof(ethercat_hardware::MotorHeatingModelParametersEepromConfig) == 256);
  BOOST_STATIC_ASSERT( offsetof(ethercat_hardware::MotorHeatingModelParametersEepromConfig, crc32_) == (256-4));
  boost::crc_32_type crc32;  
  crc32.process_bytes(this, offsetof(MotorHeatingModelParametersEepromConfig, crc32_));
  return (this->crc32_ == crc32.checksum());
}

void MotorHeatingModelParametersEepromConfig::generateCRC(void)
{
  boost::crc_32_type crc32;
  crc32.process_bytes(this, offsetof(MotorHeatingModelParametersEepromConfig, crc32_));
  this->crc32_ = crc32.checksum();
}


bool MotorHeatingModel::loaded_rosparams_ = false;
std::string MotorHeatingModel::save_directory_("/var/lib/motor_heating_model");
bool MotorHeatingModel::do_not_load_save_files_ = false;
bool MotorHeatingModel::do_not_halt_ = false;


  /*! \brief Constructor
   * 
   * \param motor_params Heating parameters used by motor heating model
   * \param actuator_name Name of actuator (ex: fl_caster_roataion_motor)
   * \param save_directory Directory where motor temperature will be routinely saved
   * \param device_position Position of device on EtherCAT chain.  Used to prevent multiple motor models from saving at same time  
   * 
   */
MotorHeatingModel::MotorHeatingModel(const MotorHeatingModelParameters &motor_params,
                                     const std::string &actuator_name,
                                     const std::string &hwid,
                                     unsigned device_position
                                     ) :
  overheat_(false),
  publisher_(NULL),
  diag_cycles_since_last_save_(0),
  motor_params_(motor_params),
  actuator_name_(actuator_name),
  hwid_(hwid)
{


  // There are a couple of rosparams that can be used to control the motor heating motor
  //  * <namespace>/motor_heating_model/do_not_load_save_files : 
  //      boolean : defaults to true
  //      Do not use saved temperature data for motor.  
  //      This might be useful for things like the the burn-in test stands 
  //      where different parts are constantly switched in-and-out.
  //  * <namespace>/motor_heating_model/save_directory : 
  //      string, defaults to '/var/lib'
  //      Location where motor heating model save data is loaded from and saved to
  //  * <namespace>/motor_heating_model/do_not_halt : 
  //      boolean, defaults to false 
  //      
  if (!loaded_rosparams_)
  {
    //save_directory_
    loaded_rosparams_ = true;  
    
    if (!boost::filesystem::exists(save_directory_))
    {
      ROS_WARN("Motor heating motor save directory '%s' does not exist, creating it", save_directory_.c_str());
      try {
        boost::filesystem::create_directory(save_directory_);
      } 
      catch (const std::exception &e)
      {
        ROS_ERROR("Error creating save directory '%s' for motor model : %s", 
                  save_directory_.c_str(), e.what());
      }
    }
  }

  save_filename_ = save_directory_ + "/" + actuator_name_ + ".save";

  // If the guess is too low, the motors may not halt when they get too hot.
  // If the guess is too high the motor may halt when they should not.

  // The first time the motor heating model is run on new machine, there will be no saved motor temperature data.
  // With the saved temperatures, we have to 'guess' an initial motor temperature. 
  // Because the motor have been used for multiple years with only a couple incidents, 
  // its probably better to error on the side of false negitives.
  static const double default_ambient_temperature = 60.0;
  winding_temperature_ = default_ambient_temperature;
  housing_temperature_ = default_ambient_temperature;
  ambient_temperature_ = default_ambient_temperature;

  // Calculate internal parameters from motor parameters
  winding_to_housing_thermal_conductance_ = 1.0 / motor_params_.winding_to_housing_thermal_resistance_;
  housing_to_ambient_thermal_conductance_ = 1.0 / motor_params_.housing_to_ambient_thermal_resistance_;  
  winding_thermal_mass_inverse_ = 
    motor_params_.winding_to_housing_thermal_resistance_ / motor_params_.winding_thermal_time_constant_;
  housing_thermal_mass_inverse_ = 
    motor_params_.housing_to_ambient_thermal_resistance_ / motor_params_.housing_thermal_time_constant_; 

  // TODO, is it actually worth the overhead to have ROS node publishing motor heating information?
  // possibly remove realtime publisher after testing of motor model has been completed.
  std::string topic("motor_temperature");
  if (!actuator_name_.empty())
  {
    topic = topic + "/" + actuator_name_;
    publisher_ = new realtime_tools::RealtimePublisher<ethercat_hardware::MotorTemperature>(ros::NodeHandle(), topic, 1, true);
    if (publisher_ == NULL)
    {
      ROS_ERROR("Could not allocate realtime publisher");
    }
  }
  
  // Currently motor model data is saved as part of diagnostic publishing function.   
  // The diagnostic publishing function is called at 1Hz.  
  // The motor model data is saved after every N cycles. 
  // Use device position to avoid having all devices save during same diagnostic cycle
  diag_cycles_since_last_save_ = device_position % DIAG_CYCLES_BETWEEN_SAVES;

  // have motor heating model load last saved temperaures from filesystem
  if (!loadTemperatureState())
  {
    ROS_WARN("Could not load motor temperature state for %s", actuator_name_.c_str());
  }

  // TODO : maybe in future is would be better to have a shared thread for all 
  // motor_heating_model classes that will save data for all devices.

}



double MotorHeatingModel::calculateMotorHeatPower(const ethercat_hardware::MotorTraceSample &s, 
                                                  const ethercat_hardware::ActuatorInfo &ai)
{
  // Normally the power being put into motor is I^2*R.  
  // However, the motor resistance changes with temperature
  // Instead this fuction breaks motor voltage into two parts
  //   1. back-EMF voltage
  //   2. resistance voltage
  // Then the power being put into motor is (resistance voltage * I)

  // Output voltage of MCB.  Guesstimate by multipling pwm ratio with supply voltage
  double output_voltage   = s.programmed_pwm * s.supply_voltage;
  // Back emf voltage of motor
  double back_emf_voltage = s.velocity * ai.speed_constant;
  // What ever voltage is left over must be the voltage used to drive current through the motor
  double resistance_voltage  = output_voltage - back_emf_voltage;

  // Power going into motor as heat (Watts)
  double heating_power = resistance_voltage * s.measured_current;
  return heating_power;
}


bool MotorHeatingModel::update(double heating_power, double ambient_temperature, double duration)
{
  // motor winding gains heat power (from motor current)
  // motor winding losses heat to motor housing
  // motor housing gets heat from winding and looses heat to ambient
  double heating_energy = heating_power * duration;
  double winding_energy_loss = 
    (winding_temperature_ - housing_temperature_) * winding_to_housing_thermal_conductance_ * duration;
  double housing_energy_loss = 
    (housing_temperature_ - ambient_temperature) * housing_to_ambient_thermal_conductance_ * duration;

  winding_temperature_ += (heating_energy      - winding_energy_loss) * winding_thermal_mass_inverse_;
  housing_temperature_ += (winding_energy_loss - housing_energy_loss) * housing_thermal_mass_inverse_;

  { // LOCKED
    boost::lock_guard<boost::mutex> lock(mutex_);
    heating_energy_sum_ += heating_energy;
    ambient_temperature_sum_ += (ambient_temperature * duration);
    duration_since_last_sample_ += duration;
    if (winding_temperature_ > motor_params_.max_winding_temperature_)
      overheat_ = true;
  } // LOCKED

  return !overheat_;
}


void MotorHeatingModel::updateFromDowntime(double downtime, double saved_ambient_temperature)
{
  ROS_WARN("Initial temperatures : winding  = %f, housing = %f", winding_temperature_, housing_temperature_);
  double saved_downtime = downtime;

  ros::Time start = ros::Time::now();

  static const double heating_power = 0.0; // While motor was off heating power should be zero

  // Simulate motor heating model with incrementally increasing simulation timestep interval
  double interval = 0.1;
  for (int i=0; i<200; ++i)
  {
    if (downtime > interval)
    {
      update(heating_power, saved_ambient_temperature, interval);
      downtime -= interval;
    }
    else
    {
      update(heating_power, saved_ambient_temperature, downtime);
      downtime = 0.0;
      break;
    }
    interval *= 1.04; // increase interval by 4% every cycle
  }

  // After 200 cycles (1.8 hours of simulation time), assume motor has cooled to ambient
  if (downtime > 0.0)
  {
    winding_temperature_ = saved_ambient_temperature;
    housing_temperature_ = saved_ambient_temperature;
  }

  ros::Time stop = ros::Time::now();

  ROS_WARN("Final temperatures : winding  = %f, housing = %f", winding_temperature_, housing_temperature_);
  ROS_WARN("Took %f milliseconds to sim %f seconds", (stop-start).toSec()*1000., saved_downtime);
}


void MotorHeatingModel::reset()
{ 
  { // LOCKED
    boost::lock_guard<boost::mutex> lock(mutex_);
    overheat_ = false;
  } // LOCKED
}


void MotorHeatingModel::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
{
  // Sample motor temperature and heating energy every so often, this way the 
  // heating of the motor can be published when there is a problem
  bool overheat;
  double winding_temperature;
  double housing_temperature;
  double ambient_temperature_sum;
  double heating_energy_sum;
  double duration_since_last_sample;
  double average_ambient_temperature;

  { // LOCKED
    boost::lock_guard<boost::mutex> lock(mutex_);
    overheat = overheat_;
    winding_temperature = winding_temperature_;
    housing_temperature = housing_temperature_;
    ambient_temperature_sum = ambient_temperature_sum_;
    heating_energy_sum  = heating_energy_sum_;
    duration_since_last_sample = duration_since_last_sample_;

    if (duration_since_last_sample > 0.0)
    {
      average_ambient_temperature = ambient_temperature_sum / duration_since_last_sample;
      ambient_temperature_ = average_ambient_temperature;
    }
    else 
    {
      average_ambient_temperature = ambient_temperature_;
    }
    
    ambient_temperature_sum_ = 0.0;
    heating_energy_sum  = 0.0;
    duration_since_last_sample_ = 0.0;
  } // LOCKED


  double average_heating_power       = heating_energy_sum / duration_since_last_sample;

  const int ERROR = 2;
  const int WARN = 1;
  //const int GOOD = 0;

  if (overheat)
  {
    // Once motor overheating flag is set, keep reporting motor has overheated, until the flag is reset.
    d.mergeSummary(ERROR, "Motor overheated");
  }
  else if (winding_temperature > (motor_params_.max_winding_temperature_ * 0.90))
  {
    // If motor winding temperature reaches 90% of limit, then make a warning
    d.mergeSummary(WARN, "Motor hot");
  }
  
  d.addf("Motor winding temp limit (C)", "%f", motor_params_.max_winding_temperature_);
  d.addf("Motor winding temp (C)", "%f", winding_temperature);
  d.addf("Motor housing temp (C)", "%f", housing_temperature);

  // TODO: removing this, probably not need for purposes other than debugging
  d.addf("Heating power (Watts)", "%f", average_heating_power);
  d.addf("Ambient temp (C)", "%f", average_ambient_temperature);

  
  if ((publisher_ != NULL) && (publisher_->trylock()))
  {    
    // Fill in sample values
    ethercat_hardware::MotorTemperature &msg(publisher_->msg_);
    msg.stamp = ros::Time::now();
    msg.winding_temperature = winding_temperature;
    msg.housing_temperature = housing_temperature;
    msg.ambient_temperature = average_ambient_temperature;
    msg.heating_power       = average_heating_power;

    // publish sample
    publisher_->unlockAndPublish();
  }

  // Save motor heating model information every 60 seconds
  if (++diag_cycles_since_last_save_ > DIAG_CYCLES_BETWEEN_SAVES)
  {
    diag_cycles_since_last_save_ = 0;
    saveTemperatureState();
  }

}



static bool getStringAttribute(TiXmlElement *elt, const std::string& filename, const char* param_name, std::string &value)
{
  const char *val_str = elt->Attribute(param_name);
  if (val_str == NULL)
  {
    ROS_ERROR("No '%s' attribute for actuator '%s'", param_name, filename.c_str());
    return false;
  }
  value = val_str;
  return true;
}


static bool getDoubleAttribute(TiXmlElement *elt, const std::string& filename, const char* param_name, double &value)
{
  const char *val_str = elt->Attribute(param_name);
  if (val_str == NULL)
  {
    ROS_ERROR("No '%s' attribute in '%s'", param_name, filename.c_str());
    return false;
  }

  char *endptr=NULL;
  value = strtod(val_str, &endptr);
  if ((endptr == val_str) || (endptr < (val_str+strlen(val_str))))
  {
    ROS_ERROR("Couldn't convert '%s' to double for attribute '%s' in '%s'", 
              val_str, param_name, filename.c_str());
    return false;
  }

  return true;
}


static void saturateTemperature(double &temperature, const char *name)
{
  static const double max_realistic_temperature = 200.0;
  static const double min_realistic_temperature = -10.0;

  if (temperature > max_realistic_temperature)
  {
    ROS_WARN("%s temperature of %f Celcius is unrealisic. Using %f instead",
                name, temperature, max_realistic_temperature);
    temperature = max_realistic_temperature;
  }
  if (temperature < min_realistic_temperature)
  {
    ROS_WARN("%s temperature of %f Celcius is unrealisic. Using %f instead",
                name, temperature, min_realistic_temperature);
    temperature = min_realistic_temperature;
  }
}


bool MotorHeatingModel::loadTemperatureState()
{
  /*
   * The tempature save file will be named after the actuator it was created for
   *   <actuactor_name>_motor_heating_model.save
   *
   * The temperatureState file will have a XML format:
   * <motor_heating_model
   *   version = "1"
   *   actuator_name = "fl_caster_r_wheel_motor"
   *   winding_temperature = "100.0"
   *   housing_temperature = "100.0" 
   *   save_time = "<wall_time when file was lasted saved>"
   *   checksum  = "<CRC32 of all previous lines in file>, to allow user editing of files checksum can be "IGNORE-CRC"
   * />
   *
   */

  if (!boost::filesystem::exists(save_filename_))
  {
    ROS_WARN("Motor heating model saved file '%s' does not exist.  Using defaults", save_filename_.c_str());
    return false;
  }

  TiXmlDocument xml;
  if (!xml.LoadFile(save_filename_))
  {
    ROS_ERROR("Unable to parse XML in save file '%s'", save_filename_.c_str());
    return false;
  }

  
  TiXmlElement *motor_temp_elt = xml.RootElement();
  if (motor_temp_elt == NULL)
  {
    ROS_ERROR("Unable to parse XML in save file '%s'", save_filename_.c_str());
    return false;
  }
      
  std::string version;
  std::string actuator_name;
  std::string hwid;
  double winding_temperature;
  double housing_temperature;
  double ambient_temperature;
  double save_time;
  if (!getStringAttribute(motor_temp_elt, save_filename_, "version", version))
  {
    return false;
  }
  const char* expected_version = "1";
  if (version != expected_version)
  {
    ROS_ERROR("Unknown version '%s', expected '%s'", version.c_str(), expected_version);
    return false;
  }
  
  bool success = true;
  success &= getStringAttribute(motor_temp_elt, save_filename_, "actuator_name", actuator_name);
  success &= getStringAttribute(motor_temp_elt, save_filename_, "hwid", hwid);
  success &= getDoubleAttribute(motor_temp_elt, save_filename_, "housing_temperature", housing_temperature);
  success &= getDoubleAttribute(motor_temp_elt, save_filename_, "winding_temperature", winding_temperature);
  success &= getDoubleAttribute(motor_temp_elt, save_filename_, "ambient_temperature", ambient_temperature);
  success &= getDoubleAttribute(motor_temp_elt, save_filename_, "save_time", save_time);  
  if (!success)
  {
    return false;
  }

  if (actuator_name != actuator_name_)
  {
    ROS_ERROR("In save file '%s' : expected actuator name '%s', got '%s'", 
              save_filename_.c_str(), actuator_name_.c_str(), actuator_name.c_str());
    return false;
  }  

  if (hwid != hwid_)
  {
    ROS_WARN("In save file '%s' : expected HWID '%s', got '%s'", 
              save_filename_.c_str(), hwid_.c_str(), hwid.c_str());    
  }

  saturateTemperature(housing_temperature, "Housing");
  saturateTemperature(winding_temperature, "Winding");
  saturateTemperature(ambient_temperature, "Ambient");

  // Update stored temperatures
  winding_temperature_ = winding_temperature;
  housing_temperature_ = housing_temperature;
  ambient_temperature_ = ambient_temperature;

  // Update motor temperature model based on saved time, update time, and saved ambient temperature
  double downtime = ros::Time::now().toSec() - save_time;
  if (downtime < 0.0)
  {
    ROS_WARN("In save file '%s' : save time is %f seconds in future", save_filename_.c_str(), -downtime);
  }
  else
  {
    updateFromDowntime(downtime, ambient_temperature);
  }

  // Make sure simulation didn't go horribly wrong
  saturateTemperature(housing_temperature_, "(2) Housing");
  saturateTemperature(winding_temperature_, "(2) Winding");

  return true;
}





bool MotorHeatingModel::saveTemperatureState()
{
  /*
   * This will first generate new XML data into temp file, then rename temp file to final filename
   *
   */

  ROS_WARN("saving motor heating model state");

  std::string tmp_filename = save_filename_ + ".tmp";

  double winding_temperature;
  double housing_temperature;
  double ambient_temperature;
  { // LOCKED
    boost::lock_guard<boost::mutex> lock(mutex_);
    winding_temperature = winding_temperature_;
    housing_temperature = housing_temperature_;
    ambient_temperature = ambient_temperature_;
  } // LOCKED

  TiXmlDocument xml;
  TiXmlDeclaration *decl = new TiXmlDeclaration( "1.0", "", "" );
  TiXmlElement *elmt = new TiXmlElement("motor_heating_model");
  elmt->SetAttribute("version", "1");
  elmt->SetAttribute("actuator_name", actuator_name_);
  elmt->SetAttribute("hwid", hwid_);
  elmt->SetDoubleAttribute("winding_temperature", winding_temperature);
  elmt->SetDoubleAttribute("housing_temperature", housing_temperature);
  elmt->SetDoubleAttribute("ambient_temperature", ambient_temperature);
  elmt->SetDoubleAttribute("save_time", ros::Time::now().toSec() );

  xml.LinkEndChild(decl);
  xml.LinkEndChild( elmt );

  // save xml with temporary filename
  if (!xml.SaveFile(tmp_filename))
  {
    ROS_WARN("Saving motor heating model file '%s'", tmp_filename.c_str());
    return false;
  }

  // now rename file
  if (rename(tmp_filename.c_str(), save_filename_.c_str()) != 0)
  {
    int error = errno;
    char errbuf[100];
    strerror_r(error, errbuf, sizeof(errbuf));
    errbuf[sizeof(errbuf)-1] = '\0';
    ROS_WARN("Renaming '%s' to '%s' : (%d) '%s'", 
             tmp_filename.c_str(), save_filename_.c_str(), error, errbuf);
    return false;
  }

  return true;
}



};  // end namespace ethercat_hardware
