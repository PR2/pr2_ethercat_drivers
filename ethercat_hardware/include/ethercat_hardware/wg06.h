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

#ifndef ETHERCAT_HARDWARE_WG06_H
#define ETHERCAT_HARDWARE_WG06_H

#include <ethercat_hardware/wg0x.h>

#include <ethercat_hardware/wg_soft_processor.h>

#include <pr2_msgs/PressureState.h>
#include <pr2_msgs/AccelerometerState.h>
#include <ethercat_hardware/RawFTData.h>
#include <geometry_msgs/WrenchStamped.h>

struct WG06StatusWithAccel
{
  uint8_t mode_;
  uint8_t digital_out_;
  int16_t programmed_pwm_value_;
  int16_t programmed_current_;
  int16_t measured_current_;
  uint32_t timestamp_;
  int32_t encoder_count_;
  int32_t encoder_index_pos_;
  uint16_t num_encoder_errors_;
  uint8_t encoder_status_;
  uint8_t unused1;
  int32_t unused2;
  int32_t unused3;
  uint16_t board_temperature_;
  uint16_t bridge_temperature_;
  uint16_t supply_voltage_;
  int16_t motor_voltage_;
  uint16_t packet_count_;
  uint8_t pad_;
  uint8_t accel_count_;
  uint32_t accel_[4];
  uint8_t checksum_;

  static const unsigned SIZE=61;
}__attribute__ ((__packed__));


struct FTDataSample
{
  int16_t data_[6];
  uint16_t vhalf_;
  uint8_t sample_count_;
  uint8_t timestamp_;
  static const unsigned SIZE=16;
}__attribute__ ((__packed__));


class FTParamsInternal
{
public:
  FTParamsInternal();

  const double &calibration_coeff(unsigned row, unsigned col) const {return calibration_coeff_[row*6 + col];}
  double &calibration_coeff(unsigned row, unsigned col) {return calibration_coeff_[row*6 + col];}

  const double &offset(unsigned ch_num) const {return offsets_[ch_num];}
  double &offset(unsigned ch_num) {return offsets_[ch_num];}

  const double &gain(unsigned ch_num) const {return gains_[ch_num];}
  double &gain(unsigned ch_num) {return gains_[ch_num];}
  
  void print() const;

  bool getRosParams(ros::NodeHandle nh);
  bool getDoubleArray(XmlRpc::XmlRpcValue params, const char* name, double *results, unsigned len);
  double calibration_coeff_[36];
  double offsets_[6];
  double gains_[6];
};


struct WG06StatusWithAccelAndFT
{
  uint8_t mode_;
  uint8_t digital_out_;
  int16_t programmed_pwm_value_;
  int16_t programmed_current_;
  int16_t measured_current_;
  uint32_t timestamp_;
  int32_t encoder_count_;
  int32_t encoder_index_pos_;
  uint16_t num_encoder_errors_;
  uint8_t encoder_status_;
  uint8_t unused1;
  int32_t unused2;
  int32_t unused3;
  uint16_t board_temperature_;
  uint16_t bridge_temperature_;
  uint16_t supply_voltage_;
  int16_t motor_voltage_;
  uint16_t packet_count_;
  uint8_t pad_;
  uint8_t accel_count_;
  uint32_t accel_[4];
  uint8_t unused4[3];
  uint8_t ft_sample_count_;
  FTDataSample ft_samples_[4];
  uint8_t checksum_;

  static const unsigned SIZE=129;
}__attribute__ ((__packed__));


struct WG06Pressure
{
  uint32_t timestamp_;
  uint16_t l_finger_tip_[22];
  uint16_t r_finger_tip_[22];
  uint8_t pad_;
  uint8_t checksum_;
  static const unsigned SIZE=94;
} __attribute__((__packed__));


struct WG06BigPressure
{
  WG06Pressure pressure_;
  uint8_t pad_[418];
  uint8_t checksum_;
  static const unsigned SIZE=513;
} __attribute__((__packed__));



class WG06 : public WG0X
{
public:
  WG06();
  ~WG06();
  int initialize(pr2_hardware_interface::HardwareInterface *, bool allow_unprogrammed=true);
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  virtual void multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer);
  enum
  {
    PRODUCT_CODE = 6805006
  };
private:

  static const unsigned PRESSURE_PHY_ADDR     = 0x2200;
  static const unsigned BIG_PRESSURE_PHY_ADDR = 0x2600;

  pr2_hardware_interface::PressureSensor pressure_sensors_[2];
  pr2_hardware_interface::Accelerometer accelerometer_;

  bool initializePressure(pr2_hardware_interface::HardwareInterface *hw);
  bool initializeAccel(pr2_hardware_interface::HardwareInterface *hw);
  bool initializeFT(pr2_hardware_interface::HardwareInterface *hw);
  bool initializeSoftProcessor();

  bool unpackPressure(unsigned char* pressure_buf);
  bool unpackAccel(WG06StatusWithAccel *status, WG06StatusWithAccel *last_status);
  bool unpackFT(WG06StatusWithAccelAndFT *status, WG06StatusWithAccelAndFT *last_status);

  void diagnosticsWG06(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  void diagnosticsAccel(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);
  void diagnosticsFT(diagnostic_updater::DiagnosticStatusWrapper &d, WG06StatusWithAccelAndFT *status);
  void diagnosticsPressure(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  //! True if device has accelerometer and force/torque sensor
  bool has_accel_and_ft_;

  bool pressure_checksum_error_; //!< Set true where checksum error on pressure data is detected, cleared on reset
  unsigned pressure_checksum_error_count_; //!< debugging
  unsigned pressure_size_; //!< Size in bytes of pressure data region

  unsigned accelerometer_samples_; //!< Number of accelerometer samples since last publish cycle
  unsigned accelerometer_missed_samples_;  //!< Total of accelerometer samples that were missed
  ros::Time last_publish_time_; //!< Time diagnostics was last published
  bool first_publish_; 

  static const unsigned NUM_PRESSURE_REGIONS = 22;    
  uint32_t last_pressure_time_;
  realtime_tools::RealtimePublisher<pr2_msgs::PressureState> *pressure_publisher_;
  realtime_tools::RealtimePublisher<pr2_msgs::AccelerometerState> *accel_publisher_;

  void convertFTDataSampleToWrench(const FTDataSample &sample, geometry_msgs::Wrench &wrench);
  static const unsigned MAX_FT_SAMPLES = 4;  
  static const unsigned NUM_FT_CHANNELS = 6;
  static const int FT_VHALF_IDEAL = 32768; //!< Vhalf ADC measurement is ideally about (1<<16)/2
  static const int FT_VHALF_RANGE = 300;  //!< allow vhalf to range +/- 300 from ideal
  int      ft_overload_limit_; //!< Limit on raw range of F/T input 
  uint8_t  ft_overload_flags_;  //!< Bits 0-5 set to true if raw FT input goes beyond limit
  bool     ft_disconnected_;  //!< f/t sensor may be disconnected
  bool     ft_vhalf_error_; //!< error with Vhalf reference voltage
  bool     ft_sampling_rate_error_; //!< True if FT sampling rate was incorrect
  uint64_t ft_sample_count_;  //!< Counts number of ft sensor samples
  uint64_t ft_missed_samples_;  //!< Counts number of ft sensor samples that were missed
  uint64_t diag_last_ft_sample_count_; //!< F/T Sample count last time diagnostics was published
  pr2_hardware_interface::AnalogIn ft_raw_analog_in_;  //!< Provides raw F/T data to controllers
  //! Provides F/T data to controllers (deprecated, use pr2_hardware_interface::ForceTorque instead)
  pr2_hardware_interface::AnalogIn ft_analog_in_;  
  //! Provides F/T data to controllers
  pr2_hardware_interface::ForceTorque force_torque_;

  //! Realtime Publisher of RAW F/T data 
  realtime_tools::RealtimePublisher<ethercat_hardware::RawFTData> *raw_ft_publisher_;
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> *ft_publisher_;
  //pr2_hardware_interface::AnalogIn ft_analog_in_;      //!< Provides
  FTParamsInternal ft_params_;

  bool enable_pressure_sensor_;
  bool enable_ft_sensor_;

  /** Certain version of WG06 firmware (2.xx and 3.xx) use soft-processors to 
   *  communicate with certain peripherals (pressure sensor, F/T sensor, accelerometer...)
   *  For purposes of supporting new types of devices, the soft-processor FW can be be
   *  modified through use of service calls.   
   */
  bool enable_soft_processor_access_;
  WGSoftProcessor soft_processor_;
};

#endif /* ETHERCAT_HARDWARE_WG06_H */
