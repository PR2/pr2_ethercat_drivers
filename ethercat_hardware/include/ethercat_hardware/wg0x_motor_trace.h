#ifndef WG0X_MOTOR_TRACE_H
#define WG0X_MOTOR_TRACE_H

#include <string>
#include <vector>

#include <realtime_tools/realtime_publisher.h>
#include <ethercat_hardware/MotorTraceSample.h>
#include <ethercat_hardware/MotorTrace.h>
#include <ethercat_hardware/ActuatorInfo.h>
#include <ethercat_hardware/BoardInfo.h>

class WG0XMotorTrace
{
public:
  WG0XMotorTrace(unsigned max_size);
  bool initialize(const ethercat_hardware::ActuatorInfo &actuator_info, const ethercat_hardware::BoardInfo &board_info);
  void publish();
  void add_sample(const ethercat_hardware::MotorTraceSample &s);
protected:
  unsigned max_size_;
  unsigned index_;  /* index of most recent element in trace buffer */
  ethercat_hardware::ActuatorInfo actuator_info_;
  ethercat_hardware::BoardInfo board_info_;
  std::vector<ethercat_hardware::MotorTraceSample> trace_buffer_;
  realtime_tools::RealtimePublisher<ethercat_hardware::MotorTrace> *publisher_;
};

#endif /* WG0X_MOTOR_TRACE_H */
