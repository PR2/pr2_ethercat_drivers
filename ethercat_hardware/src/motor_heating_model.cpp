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


MotorHeatingModel::MotorHeatingModel(const MotorHeatingModelParameters &motor_params,
                                     const std::string &actuator_name) :
  publisher_(NULL),
  motor_params_(motor_params),
  actuator_name_(actuator_name)
{
  // Ideally, winding and housing temperature will be loading from save file later.  
  // For now assume temperature starts at max
  winding_temperature_ = motor_params_.max_winding_temperature_;
  housing_temperature_ = motor_params_.max_winding_temperature_;

  // Calculate internal parameters from motor parameters
  winding_to_housing_thermal_conductance_ = 1.0 / motor_params_.winding_to_housing_thermal_resistance_;
  housing_to_ambient_thermal_conductance_ = 1.0 / motor_params_.housing_to_ambient_thermal_resistance_;  
  winding_thermal_mass_inverse_ = 
    motor_params_.winding_to_housing_thermal_resistance_ / motor_params_.winding_thermal_time_constant_;
  housing_thermal_mass_inverse_ = 
    motor_params_.housing_to_ambient_thermal_resistance_ / motor_params_.housing_thermal_time_constant_; 

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

  {
    // TODO : LOCK
    heating_energy_sum_ += heating_energy;
    ambient_temperature_sum_ += (ambient_temperature * duration);
    duration_since_last_sample_ += duration;
    if (winding_temperature_ > motor_params_.max_winding_temperature_)
      overheat_ = true;
  }

  return !overheat_;
}


void MotorHeatingModel::updateFromDowntime(ros::Duration downtime)
{
  // TODO
}

void MotorHeatingModel::reset()
{  
  //TODO LOCK
  overheat_ = false;
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
  {
    // TODO LOCK
    overheat = overheat_;
    winding_temperature = winding_temperature_;
    housing_temperature = housing_temperature_;
    ambient_temperature_sum = ambient_temperature_sum_;
    heating_energy_sum  = heating_energy_sum_;
    duration_since_last_sample = duration_since_last_sample_;

    ambient_temperature_sum_ = 0.0;
    heating_energy_sum  = 0.0;
    duration_since_last_sample_ = 0.0;
  }

  double average_ambient_temperature = ambient_temperature_sum / duration_since_last_sample;
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
  
  d.addf("Max motor winding temp (C)", "%f", motor_params_.max_winding_temperature_);
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
}



bool MotorHeatingModel::loadTemperatureState(const char* directory_path)
{
  // TODO
  return false;
}

};  // end namespace ethercat_hardware
