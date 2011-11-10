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

#ifndef ETHERCAT_HARDWARE__WG0X_H
#define ETHERCAT_HARDWARE__WG0X_H

#include "ethercat_hardware/ethercat_device.h"
#include "ethercat_hardware/motor_model.h"
#include "ethercat_hardware/motor_heating_model.h"
#include "realtime_tools/realtime_publisher.h"
#include "ethercat_hardware/wg_mailbox.h"
#include "ethercat_hardware/wg_eeprom.h"

#include <boost/shared_ptr.hpp>

using namespace ethercat_hardware;


struct WG0XSafetyDisableStatus
{
  uint8_t safety_disable_status_;
  uint8_t safety_disable_status_hold_;
  uint8_t safety_disable_count_;
  static const unsigned BASE_ADDR = 0xA1;
} __attribute__ ((__packed__));


struct WG0XSafetyDisableCounters
{
  uint8_t undervoltage_count_;
  uint8_t over_current_count_;
  uint8_t board_over_temp_count_;
  uint8_t bridge_over_temp_count_;
  uint8_t operate_disable_count_;
  uint8_t watchdog_disable_count_;
  static const unsigned BASE_ADDR = 0x223;
} __attribute__ ((__packed__));

struct WG0XDiagnosticsInfo
{
  int16_t config_offset_current_A_;
  int16_t config_offset_current_B_;
  uint16_t supply_current_in_;
  union {
    uint16_t supply_current_out_;
    uint16_t voltage_ref_;
  } __attribute__ ((__packed__));
  int16_t offset_current_A_;
  int16_t offset_current_B_;
  int16_t adc_current_;
  uint8_t unused1[2];
  uint8_t lowside_deadtime_;
  uint8_t highside_deadtime_;
  uint8_t unused2[14];
  uint8_t pdo_command_irq_count_;
  uint8_t mbx_command_irq_count_;
  uint8_t unused3;
  WG0XSafetyDisableCounters safety_disable_counters_;
  uint8_t unused4;
  uint8_t pdi_timeout_error_count_;
  uint8_t pdi_checksum_error_count_;
  static const unsigned BASE_ADDR = 0x200;
} __attribute__ ((__packed__));

struct WG0XConfigInfo
{
  uint32_t product_id_;
  union
  {
    uint32_t revision_;
    struct
    {
      uint8_t firmware_minor_revision_;
      uint8_t firmware_major_revision_;
      uint8_t pca_revision_;
      uint8_t pcb_revision_;
    }__attribute__ ((__packed__));
  }__attribute__ ((__packed__));
  uint32_t device_serial_number_;
  uint8_t current_loop_kp_;
  uint8_t current_loop_ki_;
  uint16_t absolute_current_limit_;
  float nominal_current_scale_;
  float nominal_voltage_scale_;
  uint8_t pad_[8];
  uint8_t configuration_status_;
  uint8_t safety_disable_status_;
  uint8_t safety_disable_status_hold_;
  uint8_t safety_disable_count_;
  uint16_t watchdog_limit_;

  static const unsigned CONFIG_INFO_BASE_ADDR = 0x0080;
}__attribute__ ((__packed__));

struct WG0XUserConfigRam
{
  uint8_t version_;
  uint8_t unused_[3];
  double zero_offset_;
  uint32_t crc32_;

  static const unsigned BASE_ADDR = 0x00C0;
}__attribute__ ((__packed__));

struct WG0XActuatorInfo
{
  uint16_t major_;              // Major revision
  uint16_t minor_;              // Minor revision
  uint32_t id_;                 // Actuator ID
  char name_[64];               // Actuator name
  char robot_name_[32];         // Robot name
  char motor_make_[32];         // Motor manufacturer
  char motor_model_[32];        // Motor model #
  double max_current_;          // Maximum current
  double speed_constant_;       // Speed constant
  double resistance_;           // Resistance
  double motor_torque_constant_; // Motor torque constant
  double encoder_reduction_;    // Reduction and sign between motor and encoder
  uint32_t pulses_per_revolution_; // # of encoder ticks per revolution
  uint8_t pad1[40];              // Pad structure to 256-4 bytes.  
  uint32_t crc32_256_;          // CRC32 of first 256-4 bytes. (minus 4 bytes for first CRC)
  uint8_t pad2[4];              // Pad structure to 264-4 bytes
  uint32_t crc32_264_;          // CRC32 over entire structure (minus 4 bytes for second CRC)

  bool verifyCRC(void) const;
  void generateCRC(void);
};

struct WG0XStatus
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
  uint8_t calibration_reading_;
  int32_t last_calibration_rising_edge_;
  int32_t last_calibration_falling_edge_;
  uint16_t board_temperature_;
  uint16_t bridge_temperature_;
  uint16_t supply_voltage_;
  int16_t motor_voltage_;
  uint16_t packet_count_;
  uint8_t pad_;
  uint8_t checksum_;

  static const unsigned SIZE=44;
}__attribute__ ((__packed__));


struct WG0XCommand
{
  uint8_t mode_;
  uint8_t digital_out_;
  int16_t programmed_pwm;
  int16_t programmed_current_;
  uint8_t pad_;
  uint8_t checksum_;
}__attribute__ ((__packed__));

struct MbxDiagnostics 
{
  MbxDiagnostics();
  uint32_t write_errors_;
  uint32_t read_errors_;
  uint32_t lock_errors_;
  uint32_t retries_;
  uint32_t retry_errors_;
};

struct WG0XDiagnostics 
{
  WG0XDiagnostics();
  void update(const WG0XSafetyDisableStatus &new_status, const WG0XDiagnosticsInfo &new_diagnostics_info);

  bool first_;
  bool valid_;
  WG0XSafetyDisableStatus safety_disable_status_;

  WG0XDiagnosticsInfo diagnostics_info_;
  
  uint32_t safety_disable_total_;
  uint32_t undervoltage_total_;
  uint32_t over_current_total_;
  uint32_t board_over_temp_total_;
  uint32_t bridge_over_temp_total_;
  uint32_t operate_disable_total_;
  uint32_t watchdog_disable_total_;

  uint32_t lock_errors_;
  uint32_t checksum_errors_;

  // Hack, use diagnostic thread to push new offset values to device
  double zero_offset_;
  double cached_zero_offset_;
};

class WG0X : public EthercatDevice
{
public:
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  WG0X();
  virtual ~WG0X();

  virtual int initialize(pr2_hardware_interface::HardwareInterface *, bool allow_unprogrammed=true);

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  bool program(EthercatCom *com, const WG0XActuatorInfo &actutor_info);
  bool program(EthercatCom *com, const MotorHeatingModelParametersEepromConfig &heating_config);

  bool readActuatorInfoFromEeprom(EthercatCom *com, WG0XActuatorInfo &actuator_info);
  bool readMotorHeatingModelParametersFromEeprom(EthercatCom *com, MotorHeatingModelParametersEepromConfig &config);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  virtual void collectDiagnostics(EthercatCom *com);

  bool publishTrace(const string &reason, unsigned level, unsigned delay);

protected:
  uint8_t fw_major_;
  uint8_t fw_minor_;
  uint8_t board_major_;  //!< Printed circuit board revision (for this value 0=='A', 1=='B')
  uint8_t board_minor_;  //!< Printed circuit assembly revision

  WG0XActuatorInfo actuator_info_;
  WG0XConfigInfo config_info_;
  double max_current_;         //!< min(board current limit, actuator current limit)

  ethercat_hardware::ActuatorInfo actuator_info_msg_; 
  static void copyActuatorInfo(ethercat_hardware::ActuatorInfo &out,  const WG0XActuatorInfo &in);

  pr2_hardware_interface::Actuator actuator_;
  pr2_hardware_interface::DigitalOut digital_out_;

  enum
  {
    MODE_OFF = 0x00,
    MODE_ENABLE = (1 << 0),
    MODE_CURRENT = (1 << 1),
    MODE_SAFETY_RESET = (1 << 4),
    MODE_SAFETY_LOCKOUT = (1 << 5),
    MODE_UNDERVOLTAGE = (1 << 6),
    MODE_RESET = (1 << 7)
  };

  enum 
  {
    WG05_PRODUCT_CODE = 6805005,
    WG06_PRODUCT_CODE = 6805006,
    WG021_PRODUCT_CODE = 6805021
  };

  static string modeString(uint8_t mode);
  static string safetyDisableString(uint8_t status);
  bool in_lockout_;
  bool resetting_;
  bool has_error_;
  uint16_t max_bridge_temperature_, max_board_temperature_;
  bool too_many_dropped_packets_;
  bool status_checksum_error_;
  bool timestamp_jump_detected_;
  bool fpga_internal_reset_detected_;
  bool encoder_errors_detected_;

  void clearErrorFlags(void);

  double cached_zero_offset_;
  enum {NO_CALIBRATION=0, CONTROLLER_CALIBRATION=1, SAVED_CALIBRATION=2};
  int calibration_status_;

  /** The ros::Duration timestamp measures the time since the ethercat process started.
   * It is generated by accumulating diffs between 32bit device timestamps.
   * Device timestamps are assumed to be in microseconds, so 32bit timestamp will overflow every 72 minutes,
   * ros::Duration (int32_t secs, int32_t nsecs) should overflow will overflow after 68 years
   */
  ros::Duration sample_timestamp_;
  
  //! Different possible states for application ram on device. 
  //  Application ram is non-volitile memory that application can use to store temporary
  //  data between runs.
  //  Currently app ram is used to store zero offset of joint after calibration
  //    PRESENT - App ram is available
  //    MISSING - App ram is missing but would typically be availble
  //    NOT_APPLICABLE - App ram is not availble, but is is not needed for this type of device 
  enum AppRamStatus { APP_RAM_PRESENT=1, APP_RAM_MISSING=2, APP_RAM_NOT_APPLICABLE=3 };
  AppRamStatus app_ram_status_; 
  bool readAppRam(EthercatCom *com, double &zero_offset);
  bool writeAppRam(EthercatCom *com, double zero_offset);

  bool verifyState(WG0XStatus *this_status, WG0XStatus *prev_status);

  int writeMailbox(EthercatCom *com, unsigned address, void const *data, unsigned length);
  int readMailbox(EthercatCom *com, unsigned address, void *data, unsigned length);  

  void publishGeneralDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);
  void publishMailboxDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);

  bool initializeMotorModel(pr2_hardware_interface::HardwareInterface *hw, 
                            const string &device_description,
                            double max_pwm_ratio, 
                            double board_resistance,
                            bool poor_measured_motor_voltage);

  bool initializeMotorHeatingModel(bool allow_unprogrammed);

  bool verifyChecksum(const void* buffer, unsigned size);
  static bool timestamp_jump(uint32_t timestamp, uint32_t last_timestamp, uint32_t amount);
  
  static const int PWM_MAX = 0x4000;
  
protected:
  //! Mailbox access to device
  ethercat_hardware::WGMailbox mailbox_;

  //! Access to device eeprom
  ethercat_hardware::WGEeprom eeprom_;

  static const unsigned COMMAND_PHY_ADDR = 0x1000;
  static const unsigned STATUS_PHY_ADDR = 0x2000;

  static const unsigned PDO_COMMAND_SYNCMAN_NUM = 0;
  static const unsigned PDO_STATUS_SYNCMAN_NUM  = 1;

  enum
  {
    LIMIT_SENSOR_0_STATE = (1 << 0),
    LIMIT_SENSOR_1_STATE = (1 << 1),
    LIMIT_ON_TO_OFF = (1 << 2),
    LIMIT_OFF_TO_ON = (1 << 3)
  };

  enum
  {
    SAFETY_DISABLED = (1 << 0),
    SAFETY_UNDERVOLTAGE = (1 << 1),
    SAFETY_OVER_CURRENT = (1 << 2),
    SAFETY_BOARD_OVER_TEMP = (1 << 3),
    SAFETY_HBRIDGE_OVER_TEMP = (1 << 4),
    SAFETY_OPERATIONAL = (1 << 5),
    SAFETY_WATCHDOG = (1 << 6)
  };

  // Board configuration parameters

  static const unsigned ACTUATOR_INFO_PAGE = 4095;

  // Not all devices will need this (WG021 won't) 
  MotorModel *motor_model_; 
  bool disable_motor_model_checking_;
  ethercat_hardware::MotorTraceSample motor_trace_sample_;
  pr2_hardware_interface::DigitalOut publish_motor_trace_; 

  // Only device with motor heating parameters store in eeprom config will use this
  static boost::shared_ptr<ethercat_hardware::MotorHeatingModelCommon> motor_heating_model_common_;
  boost::shared_ptr<ethercat_hardware::MotorHeatingModel> motor_heating_model_;

  // Diagnostic message values
  uint32_t last_timestamp_;
  uint32_t last_last_timestamp_;
  int drops_;
  int consecutive_drops_;
  int max_consecutive_drops_;

  bool lockWG0XDiagnostics();
  bool tryLockWG0XDiagnostics();
  void unlockWG0XDiagnostics();
  pthread_mutex_t wg0x_diagnostics_lock_;
  WG0XDiagnostics wg0x_publish_diagnostics_;
  WG0XDiagnostics wg0x_collect_diagnostics_;

public:
  static int32_t timestampDiff(uint32_t new_timestamp, uint32_t old_timestamp);
  static int32_t positionDiff(int32_t new_position, int32_t old_position);

  static ros::Duration timediffToDuration(int32_t timediff_usec);

  static double calcEncoderVelocity(int32_t new_position, uint32_t new_timestamp, 
                                    int32_t old_position, uint32_t old_timestamp);

  static double convertRawTemperature(int16_t raw_temp);
};


#endif // ETHERCAT_HARDWARE__WG0X_H
