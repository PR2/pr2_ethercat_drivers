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

#ifndef ETHERCAT_HARDWARE__MOTOR_HEATING_MODEL_H
#define ETHERCAT_HARDWARE__MOTOR_HEATING_MODEL_H

#include "ethercat_hardware/MotorTemperature.h"
#include "ethercat_hardware/MotorTraceSample.h"
#include "ethercat_hardware/ActuatorInfo.h"

#include "realtime_tools/realtime_publisher.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>

#include <vector>
#include <string>

namespace ethercat_hardware
{


/*!
 * Structure for store motor heating model parameters.   
 *
 * Parameters are based on values provided by Maxon datasheets.
 */
struct MotorHeatingModelParameters
{
  //! Thermal resistance between motor housing and ambient : in C/Watt
  double housing_to_ambient_thermal_resistance_;
  //! Thermal resistance between motor winding and motor housing : in C/Watt
  double winding_to_housing_thermal_resistance_;
  //! Thermal time constant of motor winding : in seconds
  double winding_thermal_time_constant_;
  //! Thermal time constant of motor housing : in seconds
  double housing_thermal_time_constant_;
  //! temperature limit of motor windings : in Celcius
  double max_winding_temperature_;
}  __attribute__ ((__packed__));


/*!
 * Structure for store motor heating model parameters in MCB EEPROM page.
 *
 * Eeprom pages eeprom could be 264byes or 256bytes, simplicity, fit structure to 256bytes.
 *
 * Motor model parameters could be included at part of the WG0XActuatorInfo struct, 
 * however, structure is almost full, and changing structure would require code to 
 * suppport both revisions.  Also, updating old MCBs with new parameters would 
 * not effect old parameters, making updates safer to do in the field.
 */ 
struct MotorHeatingModelParametersEepromConfig
{
  uint16_t major_;                      //!< Major revision of this structure
  uint16_t minor_;                      //!< Minor revision of this structure
  uint8_t  enforce_;                    //!< 0 if heating model should be not be enforced, 0 otherwise
  uint8_t  pad1[3];
  MotorHeatingModelParameters params_;  //!< Motor parameters  
  uint8_t  pad2[204];
  uint32_t crc32_;                      //!< CRC32 of first 256-4 bytes of structure.

  static const unsigned EEPROM_PAGE = 4093; // Acutator info on page 4095, MCB config on 4094

  bool verifyCRC(void) const;
  void generateCRC(void);
} __attribute__ ((__packed__));


class MotorHeatingModel : private boost::noncopyable
{
public:
  MotorHeatingModel(const MotorHeatingModelParameters &motor_params, const std::string &actuator_name);

  /*! Updates motor temperature estimate. based on power flowing into for 1 cycle.
   *
   * This uses motor data to determine how much heat power was put into motor 
   * over last control cycle.  The control cycle is assumed to be relatively
   * short so the update is done as a linear estimation of differential equation
   * 
   * Returns true if motor winding temperature is below acceptable limit, false if motor has overheated
   * 
   */
  bool update(double heating_power, double ambient_temperature, double duration);
  bool update(const ethercat_hardware::MotorTraceSample &sample, 
              const ethercat_hardware::ActuatorInfo &actuator_info,
              double ambient_temperature, double duration)
  {
    double heating_power = calculateMotorHeatPower(sample, actuator_info);
    return update(heating_power, ambient_temperature, duration);
  }  

  //! Resets motor overheat flag
  void reset();


  /*! Updates estimated motor temperature for long period of off-time
   *  
   * Between runs of this program the motor temperature estimate is stored 
   * to a file.  When the program is run again this saved temperature 
   * estimate needs to be updated base on time between runs.  
   * This function makes the assumption that no power was being put into motor
   * between runs.  
   *
   * Because the motor may have been disabled for a long time, this function
   * does not always use a interative approach to calculate new temperature
   */
  void updateFromDowntime(ros::Duration downtime);


  /*! Determines power being put into motor as heat (in Watts)
   */
  double calculateMotorHeatPower(const ethercat_hardware::MotorTraceSample &sample,
                                 const ethercat_hardware::ActuatorInfo &actuator_info);
  

  //! Load saved temperature estimate from directory
  bool loadTemperatureState(const char* directory_path);


  /*! Saves current temperature estimate to file given directory.
   *
   * Filename will be named : <actuator_name>_motor_temp
   * Update of file should be atomic.
   */ 
  bool saveTemperatureState(const char* directory_path);


  //! Appends heating diagnostic data to status wrapper
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);


protected:
  // Following values are calculated from motor parameters.  
  // They are more useful when peforming motor model calculations
  // 
  // Thermal is conducance amount of heat power tranfered for a 
  // given temperature differential
  // Thermal conductance units : Watt/C
  // Thermal conductance =  1 / Thermal resistance
  //
  // Thermal mass (or Thermal capicitance) : Energy required to 
  // raise temperature of object 1 degree.
  // Thermal mass units :  C/Joule
  // Thermal mass = Thermal time constant / thermal resistance  

  //! Thermal conductance between motor winding and housing : in Watt/C
  double winding_to_housing_thermal_conductance_;
  //! Thermal conductance between motor housing and ambient : in Watt/C
  double housing_to_ambient_thermal_conductance_;
  //! Inverse of thermal mass for motor winding : in Joules/C
  double winding_thermal_mass_inverse_;
  //! Inverse of thermal mass for motor housing : in Joules/C
  double housing_thermal_mass_inverse_;

  
  //! Temperature estimate of motor winding : in Celcius
  double winding_temperature_;
  //! Temperature estimate of motor housing : in Celcius
  double housing_temperature_;
  //! Last recorded ambient temperature : in Celcius
  double ambient_temperature_;

  //! Diagnostics cycles since last save
  unsigned diag_cycles_since_last_save_;

  //! True if most has overheat, once set, will only clear when reset() is called
  bool overheat_;

  //! Sum of heat energy for last sample interval
  double heating_energy_sum_;
  //! Sum of (abient heat * time) over last sample interval
  double ambient_temperature_sum_;
  //! Time (in seconds) since late sample interval occurred
  double duration_since_last_sample_;
  //! Sample interval for trace (in seconds)
  double trace_sample_interval_;
  //! realtime publisher for MotorHeatingSample
  realtime_tools::RealtimePublisher<ethercat_hardware::MotorTemperature> *publisher_;


  std::string genMotorHeatingModelSaveFilename(const char* directory_path) const;


  MotorHeatingModelParameters motor_params_;
  std::string actuator_name_;
};

}; //end namepace ethercat_hardware

#endif //ETHERCAT_HARDWARE__MOTOR_HEATING_MODEL_H
