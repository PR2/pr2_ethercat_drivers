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
#include <boost/shared_ptr.hpp>

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
  bool  enforce_;                       //!< 0 if heating model should be not be enforced, 0 otherwise
  uint8_t  pad1[3];
  MotorHeatingModelParameters params_;  //!< Motor parameters  
  uint8_t  pad2[204];
  uint32_t crc32_;                      //!< CRC32 of first 256-4 bytes of structure.
 
  static const unsigned EEPROM_PAGE = 4093; // Acutator info on page 4095, MCB config on 4094

  bool verifyCRC(void) const;
  void generateCRC(void);
} __attribute__ ((__packed__));


class MotorHeatingModel;

class MotorHeatingModelCommon : private boost::noncopyable
{
public:
  /** \brief Constructor will read motor heating mode settings from node handle namespace  
   */
  MotorHeatingModelCommon(ros::NodeHandle nh);
  
  /** \brief Constructor will use default settings for all parameters
   */
  MotorHeatingModelCommon();


  bool initialize();
  
  /**! \brief Append model to list of models that need to have temperature data saved.
   */
  void attach(boost::shared_ptr<MotorHeatingModel> model);

  //! Directory where motor model haeting data will be saved.  Defaults to /var/lib/motor_heating_model
  //! If true, then temeperature data to will be periodically saved to file
  bool update_save_files_;    
  //! Directory where temperature save files should be put
  std::string save_directory_; 
  //! If true, then class instances will attempt to load data from a saved temperature file
  bool load_save_files_; 
  //! Disables halting caused by a motor being overtemperature  
  bool disable_halt_;
  //! If true, enables motor heating model.  If false, motor heating model is run for any devices
  bool enable_model_;
  //! If true, each motor heating model with publish state information every 1 second
  bool publish_temperature_;

protected:
  bool createSaveDirectory();

  //! thread that will periodically save temperature data
  boost::thread save_thread_;
  void saveThreadFunc();

  //! List of MotorHeatingModels that need to have file data saved
  std::vector< boost::shared_ptr<MotorHeatingModel> > models_; 

  //!Lock around models list
  boost::mutex mutex_;
};



class MotorHeatingModel : private boost::noncopyable
{
public:

  MotorHeatingModel(const MotorHeatingModelParameters &motor_params,
                    const std::string &actuator_name,
                    const std::string &hwid,
                    const std::string &save_directory
                    );

  bool startTemperaturePublisher();

  /*! \brief Updates motor temperature estimate
   *
   * This uses motor data to determine how much heat power was put into motor 
   * over last control cycle.  The control cycle is assumed to be relatively
   * short so the update is done as a linear estimation of differential equation
   * 
   * Returns true if motor winding temperature is below acceptable limit, 
   *       false if motor has overheated and halting enabled. 
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

  //! Not thread save, should be called by same thread that calls update()
  bool hasOverheated() const {return overheat_;}
  //! Gets current winding temperature estimate (for testing)
  double getWindingTemperature() {return winding_temperature_;}
  //! Gets current winding temperature estimate (for testing)
  double getHousingTemperature() {return housing_temperature_;}


  //! Resets motor overheat flag
  void reset();


  /*! \brief Updates estimated motor temperature for long period of off-time
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
  void updateFromDowntime(double downtime, double saved_ambient_temperature);


  /*! \brief Updates estimated motor temperature for certain amount of downtime
   *  
   * Runs motor temperture simulation with a certain step interval and for a given 
   * number of cycles.  Will stop early if downtime reaches 0.  
   * Returns remaining downtime
   *
   */  
  double updateFromDowntimeWithInterval(double downtime, 
                                        double saved_ambient_temperature, 
                                        double interval, 
                                        unsigned cycles);


  /*! Determines power being put into motor as heat (in Watts)
   */
  double calculateMotorHeatPower(const ethercat_hardware::MotorTraceSample &sample,
                                 const ethercat_hardware::ActuatorInfo &actuator_info);
  

  //! Load saved temperature estimate from directory
  bool loadTemperatureState();


  /*! Saves current temperature estimate to file given directory.
   *
   * Filename will be named : <actuator_name>_motor_temp
   * Update of file should be atomic.
   */ 
  bool saveTemperatureState();


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

  //! mutex protects values updates by realtime thread and used by diagnostics thread  
  boost::mutex mutex_;

  //! True if most has overheat, once set, will only clear when reset() is called
  bool overheat_;

  //! Sum of heat energy for last sample interval
  double heating_energy_sum_;
  //! Sum of (abient heat * time) over last sample interval
  double ambient_temperature_sum_;
  //! Time (in seconds) since late sample interval occurred
  double duration_since_last_sample_;
  //! Sample interval for trace (in seconds)
  //double trace_sample_interval_;
  //! realtime publisher for MotorHeatingSample
  realtime_tools::RealtimePublisher<ethercat_hardware::MotorTemperature> *publisher_;

  MotorHeatingModelParameters motor_params_;
  std::string actuator_name_;  //!< name of actuator (ex. fl_caster_rotation_motor)
  std::string save_filename_;  //!< path to file where temperature data will be saved
  std::string hwid_;  //!< Hardware ID of device (ex. 680500501000)
};


}; //end namepace ethercat_hardware

#endif //ETHERCAT_HARDWARE__MOTOR_HEATING_MODEL_H
