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

#ifndef WG0X_H
#define WG0X_H

#include <ethercat_hardware/ethercat_device.h>
#include <ethercat_hardware/motor_model.h>

#include <realtime_tools/realtime_publisher.h>
#include <pr2_msgs/PressureState.h>
#include <pr2_msgs/AccelerometerState.h>

enum MbxCmdType {LOCAL_BUS_READ=1, LOCAL_BUS_WRITE=2};

struct WG0XMbxHdr
{
  uint16_t address_;
  union
  {
    uint16_t command_;
    struct
    {
      uint16_t length_:12;
      uint16_t seqnum_: 3;  // bits[14:12] sequence number, 0=disable, 1-7 normal sequence number
      uint16_t write_nread_:1;
    }__attribute__ ((__packed__));
  };
  uint8_t checksum_;

  bool build(unsigned address, unsigned length, MbxCmdType type, unsigned seqnum);
  bool verifyChecksum(void) const;
}__attribute__ ((__packed__));

static const unsigned MBX_SIZE = 512;
static const unsigned MBX_DATA_SIZE = (MBX_SIZE - sizeof(WG0XMbxHdr) - 1);
struct WG0XMbxCmd
{
  WG0XMbxHdr hdr_;
  uint8_t data_[MBX_DATA_SIZE];
  uint8_t checksum_;

  bool build(unsigned address, unsigned length, MbxCmdType type, unsigned seqnum, void const* data);
}__attribute__ ((__packed__));

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


// Syncmanger control register 0x804+N*8
struct SyncManControl {
  union {
    uint8_t raw;
    struct {
      uint8_t mode            : 2;
      uint8_t direction       : 2;
      uint8_t ecat_irq_enable : 1;
      uint8_t pdi_irq_enable  : 1;
      uint8_t watchdog_enable : 1;
      uint8_t res1            : 1;
    } __attribute__ ((__packed__));      
  } __attribute__ ((__packed__));
  //static const unsigned BASE_ADDR=0x804;
  //static unsigned base_addr(unsigned num);
  //void print(std::ostream &os=std::cout) const;
} __attribute__ ((__packed__));

// Syncmanger status register 0x805+N*8
struct SyncManStatus {
  union {
    uint8_t raw;
    struct {
      uint8_t interrupt_write : 1;
      uint8_t interrupt_read  : 1;
      uint8_t res1            : 1;
      uint8_t mailbox_status  : 1;
      uint8_t buffer_status   : 2;
      uint8_t res2            : 2;
    } __attribute__ ((__packed__));      
  } __attribute__ ((__packed__));
  //static const unsigned BASE_ADDR=0x805;
  //static unsigned base_addr(unsigned num);
  //void print(std::ostream &os=std::cout) const;
} __attribute__ ((__packed__));

// Syncmanger activation register 0x806+N*8
struct SyncManActivate {
  union {
    uint8_t raw;
    struct {
      uint8_t enable : 1;
      uint8_t repeat_request : 1;
      uint8_t res4 : 4;
      uint8_t ecat_latch_event : 1;
      uint8_t pdi_latch_event : 1;
    } __attribute__ ((__packed__));      
  } __attribute__ ((__packed__));
  static const unsigned BASE_ADDR=0x806;
  static unsigned baseAddress(unsigned num);
  //void print(std::ostream &os=std::cout) const;  
  bool writeData(EthercatCom *com, EtherCAT_SlaveHandler *sh, EthercatDevice::AddrMode addrMode, unsigned num) const;
} __attribute__ ((__packed__));

// Syncmanger PDI control register 0x807+N*8
struct SyncManPDIControl {
  union {
    uint8_t raw;
    struct {
      uint8_t deactivate : 1;
      uint8_t repeat_ack : 1;
      uint8_t res6 : 6;
    } __attribute__ ((__packed__));
  } __attribute__ ((__packed__));      
  //static const unsigned BASE_ADDR=0x807;
  //static unsigned base_addr(unsigned num);
  //void print(std::ostream &os=std::cout) const;
} __attribute__ ((__packed__));


// For SyncManager settings REG 0x800+8*N
struct SyncMan {
  union {
    uint8_t raw[8];
    struct {
      uint16_t start_addr;
      uint16_t length;
      SyncManControl control;
      SyncManStatus status;
      SyncManActivate activate;
      SyncManPDIControl pdi_control;
    } __attribute__ ((__packed__));
  } __attribute__ ((__packed__));
  
  // Base address for first syncmanager
  static const unsigned BASE_ADDR=0x800;
  // Base address of Nth syncmanager for N=0-7
  static unsigned baseAddress(unsigned num);
  
  bool readData(EthercatCom *com, EtherCAT_SlaveHandler *sh, EthercatDevice::AddrMode addrMode, unsigned num);
  //void print(unsigned num, std::ostream &os=std::cout) const;
} __attribute__ ((__packed__));


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
  uint8_t pad[48];              // Pad entire structure to 264 bytes
  uint32_t crc32_;              // CRC32 over structure (minus last 4 bytes)
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
}__attribute__ ((__packed__));

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
}__attribute__ ((__packed__));

struct WG021Status
{
  uint8_t mode_;
  uint8_t digital_out_;
  uint8_t general_config_;
  uint8_t pad1_;
  int16_t programmed_current_;
  int16_t measured_current_;
  uint32_t timestamp_;
  uint8_t config0_;
  uint8_t config1_;
  uint8_t config2_;
  uint8_t pad2_;
  uint32_t pad3_;
  uint16_t pad4_;
  uint8_t pad5_;
  uint8_t output_status_;
  uint32_t output_start_timestamp_;
  uint32_t output_stop_timestamp_;
  uint16_t board_temperature_;
  uint16_t bridge_temperature_;
  uint16_t supply_voltage_;
  int16_t led_voltage_;
  uint16_t packet_count_;
  uint8_t pad_;
  uint8_t checksum_;
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

struct WG021Command
{
  uint8_t mode_;
  uint8_t digital_out_;
  uint8_t general_config_;
  uint8_t pad1_;
  int16_t programmed_current_;
  int16_t pad2_;
  int32_t pad3_;
  uint8_t config0_;
  uint8_t config1_;
  uint8_t config2_;
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


  void program(WG0XActuatorInfo *);
  bool isProgrammed() { return actuator_info_.crc32_ != 0;}

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  virtual void collectDiagnostics(EthercatCom *com);

protected:
  uint8_t fw_major_;
  uint8_t fw_minor_;
  uint8_t board_major_;
  uint8_t board_minor_;

  WG0XActuatorInfo actuator_info_;
  WG0XConfigInfo config_info_;

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

  string reason_;
  int level_;
  string safetyDisableString(uint8_t status);
  bool in_lockout_;
  bool resetting_;
  uint16_t max_bridge_temperature_, max_board_temperature_;

  bool verifyState(WG0XStatus *this_status, WG0XStatus *prev_status);
  int readEeprom(EthercatCom *com);
  int writeEeprom(EthercatCom *com);
  int sendSpiCommand(EthercatCom *com, WG0XSpiEepromCmd const * cmd);

  int writeMailbox(EthercatCom *com, unsigned address, void const *data, unsigned length);
  int readMailbox(EthercatCom *com, unsigned address, void *data, unsigned length);  

  void publishGeneralDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);
  void publishMailboxDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);

  bool initializeMotorModel(pr2_hardware_interface::HardwareInterface *hw, 
                            const string &device_description,
                            double max_pwm_ratio, 
                            double board_resistance,
                            bool poor_measured_motor_voltage);

  static const int PWM_MAX = 0x4000;
  
private:
  // Each WG0X device can only support one mailbox operation at a time
  bool lockMailbox();
  void unlockMailbox();
  pthread_mutex_t mailbox_lock_;
  MbxDiagnostics mailbox_diagnostics_;
  MbxDiagnostics mailbox_publish_diagnostics_;

  // Mailbox helper functions
  int writeMailbox_(EthercatCom *com, unsigned address, void const *data, unsigned length);
  int readMailbox_(EthercatCom *com, unsigned address, void *data, unsigned length);  
  bool verifyDeviceStateForMailboxOperation();
  bool clearReadMailbox(EthercatCom *com);
  bool waitForReadMailboxReady(EthercatCom *com);
  bool waitForWriteMailboxReady(EthercatCom *com);
  bool readMailboxRepeatRequest(EthercatCom *com);
  bool _readMailboxRepeatRequest(EthercatCom *com);
  bool writeMailboxInternal(EthercatCom *com, void const *data, unsigned length);
  bool readMailboxInternal(EthercatCom *com, void *data, unsigned length);
  void diagnoseMailboxError(EthercatCom *com);

  static const unsigned COMMAND_PHY_ADDR = 0x1000;
  static const unsigned STATUS_PHY_ADDR = 0x2000;
  static const unsigned PRESSURE_PHY_ADDR = 0x2200;
  static const unsigned MBX_COMMAND_PHY_ADDR = 0x1400;
  static const unsigned MBX_COMMAND_SIZE = 512;
  static const unsigned MBX_STATUS_PHY_ADDR = 0x2400;
  static const unsigned MBX_STATUS_SIZE = 512;

  static const unsigned PDO_COMMAND_SYNCMAN_NUM = 0;
  static const unsigned PDO_STATUS_SYNCMAN_NUM  = 1;
  static const unsigned MBX_COMMAND_SYNCMAN_NUM = 2;
  static const unsigned MBX_STATUS_SYNCMAN_NUM  = 3;

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

  static const int ACTUATOR_INFO_PAGE = 4095;


  // Not all devices will need this (WG021) 
  MotorModel *motor_model_; 
  ethercat_hardware::MotorTraceSample motor_trace_sample_;
  pr2_hardware_interface::DigitalOut publish_motor_trace_; 

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
};

class WG05 : public WG0X
{
public:
  int initialize(pr2_hardware_interface::HardwareInterface *, bool allow_unprogrammed=true);  
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
  enum
  {
    PRODUCT_CODE = 6805005
  };
};

struct WG06Pressure
{
  uint32_t timestamp_;
  uint16_t l_finger_tip_[22];
  uint16_t r_finger_tip_[22];
  uint8_t pad_;
  uint8_t checksum_;
} __attribute__((__packed__));

class WG06 : public WG0X
{
public:
  WG06() : last_pressure_time_(0), pressure_publisher_(0), accel_publisher_(0) {}
  ~WG06();
  int initialize(pr2_hardware_interface::HardwareInterface *, bool allow_unprogrammed=true);
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  enum
  {
    PRODUCT_CODE = 6805006
  };
private:
  pr2_hardware_interface::PressureSensor pressure_sensors_[2];
  pr2_hardware_interface::Accelerometer accelerometer_;

  uint32_t last_pressure_time_;
  realtime_tools::RealtimePublisher<pr2_msgs::PressureState> *pressure_publisher_;
  realtime_tools::RealtimePublisher<pr2_msgs::AccelerometerState> *accel_publisher_;
};

class WG021 : public WG0X
{
public:
  WG021() : projector_(digital_out_A_, digital_out_B_, digital_out_I_, digital_out_M_, digital_out_L0_, digital_out_L1_) {}
  void construct(EtherCAT_SlaveHandler *sh, int &start_address) {WG0X::construct(sh, start_address);}
  int initialize(pr2_hardware_interface::HardwareInterface *, bool allow_unprogrammed=true);
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  enum
  {
    PRODUCT_CODE = 6805021
  };
  enum
  {
    PROJECTOR_CONFIG_ENABLE = 8,
    PROJECTOR_CONFIG_ENABLE_ENABLED = 8,
    PROJECTOR_CONFIG_ENABLE_DISABLED = 0,

    PROJECTOR_CONFIG_ACTION = 4,
    PROJECTOR_CONFIG_ACTION_ON = 4,
    PROJECTOR_CONFIG_ACTION_OFF = 0,

    PROJECTOR_CONFIG_POLARITY = 2,
    PROJECTOR_CONFIG_POLARITY_ACTIVE_HIGH = 2,
    PROJECTOR_CONFIG_POLARITY_ACTIVE_LOW = 0,

    PROJECTOR_CONFIG_STATE = 1,
    PROJECTOR_CONFIG_STATE_HIGH = 1,
    PROJECTOR_CONFIG_STATE_LOW = 0
  };
private:
  pr2_hardware_interface::DigitalOut digital_out_A_;
  pr2_hardware_interface::DigitalOut digital_out_B_;
  pr2_hardware_interface::DigitalOut digital_out_I_;
  pr2_hardware_interface::DigitalOut digital_out_M_;
  pr2_hardware_interface::DigitalOut digital_out_L0_;
  pr2_hardware_interface::DigitalOut digital_out_L1_;
  pr2_hardware_interface::Projector projector_;
};

#endif /* WG0X_H */
