#include <ethercat_hardware/motor_model.h>

static double max(double a, double b) {return (a>b)?a:b;}
static double min(double a, double b) {return (a<b)?a:b;}

MotorModel::MotorModel(unsigned trace_size) : 
  trace_size_(trace_size), 
  trace_index_(0),
  backemf_constant_(0.0),
  motor_voltage_error_(0.2),
  abs_motor_voltage_error_(0.02),
  measured_voltage_error_(0.2),
  abs_measured_voltage_error_(0.02),
  current_error_(0.5),
  abs_current_error_(0.02),
  abs_velocity_(0.02),
  abs_measured_current_(0.02),
  abs_board_voltage_(0.02),
  abs_position_delta_(0.02)
{
  assert(trace_size_ > 0);
  trace_buffer_.reserve(trace_size_);
}

void MotorModel::reset()
{
  filter_mutex_.lock();
  {
    motor_voltage_error_.reset();
    abs_motor_voltage_error_.reset();
    measured_voltage_error_.reset();
    abs_measured_voltage_error_.reset();
    current_error_.reset();
    abs_current_error_.reset();
    abs_velocity_.reset();  
    abs_measured_current_.reset();    
    abs_board_voltage_.reset();
    abs_position_delta_.reset();
  }
  filter_mutex_.unlock();
}

/**  \brief Initializes motor trace publisher
 */
bool MotorModel::initialize(const ethercat_hardware::ActuatorInfo &actuator_info, 
                            const ethercat_hardware::BoardInfo &board_info)
{
  std::string topic("motor_trace");
  if (!actuator_info.name.empty())
    topic = topic + "/" + actuator_info.name;
  publisher_ = new realtime_tools::RealtimePublisher<ethercat_hardware::MotorTrace>(ros::NodeHandle(), topic, 1, true);
  if (publisher_ == NULL) 
    return false;

  actuator_info_ = actuator_info;
  board_info_ = board_info;

  if (actuator_info_.speed_constant > 0.0) {
    backemf_constant_ = 1.0 / (actuator_info_.speed_constant * 2.0 * M_PI * 1.0/60.0);
  } else {
    ROS_ERROR("Invalid speed constant of %f for %s", actuator_info_.speed_constant, actuator_info_.name.c_str());
  }

  {
    ethercat_hardware::MotorTrace &msg(publisher_->msg_);
    msg.actuator_info = actuator_info;  
    msg.board_info = board_info;
    msg.samples.reserve(trace_size_);
  } 

  return true;
}

/**  \brief Publishes motor trace
 */
void MotorModel::publishTrace(const std::string &reason)
{
  assert(publisher_ != NULL);
  if ((publisher_==NULL) || (!publisher_->trylock())) 
    return;
  
  ethercat_hardware::MotorTrace &msg(publisher_->msg_);
  
  msg.header.stamp = ros::Time::now();  
  msg.reason = reason;
  unsigned size=trace_buffer_.size();
  msg.samples.clear();
  msg.samples.reserve(size);

  // TODO : is there a beter way to copy data between two std::vectors?
  for (unsigned i=0; i<size; ++i) {
    msg.samples.push_back(trace_buffer_.at((trace_index_+1+i)%size));
  }

  publisher_->unlockAndPublish();
}

/**  \brief Collects and publishes device diagnostics
 */
void MotorModel::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
{
  // Should perform locking.. publishing of diagnostics is done from separate thread
  double motor_voltage_error; 
  double motor_voltage_error_max; 
  double abs_motor_voltage_error; 
  double abs_motor_voltage_error_max; 
  double current_error; 
  double current_error_max; 
  double abs_current_error; 
  double abs_current_error_max;
  double est_motor_resistance;

  filter_mutex_.lock(); 
  {
    motor_voltage_error         = motor_voltage_error_.filter(); 
    motor_voltage_error_max     = motor_voltage_error_.filter_max(); 
    abs_motor_voltage_error     = abs_motor_voltage_error_.filter(); 
    abs_motor_voltage_error_max = abs_motor_voltage_error_.filter_max(); 
    current_error               = current_error_.filter();  
    current_error_max           = current_error_.filter_max(); 
    abs_current_error           = abs_current_error_.filter(); 
    abs_current_error_max       = abs_current_error_.filter_max(); 
    est_motor_resistance        = motor_resistance_.filter();
  }
  filter_mutex_.unlock();

  d.addf("Motor Voltage Error %", "%f",        100.0 * motor_voltage_error);
  d.addf("Max Motor Voltage Error %", "%f",    100.0 * motor_voltage_error_max);
  d.addf("Filtered Voltage Error %", "%f",     100.0 * abs_motor_voltage_error);
  d.addf("Max Filtered Voltage Error %", "%f", 100.0 * abs_motor_voltage_error_max);

  d.addf("Current Error", "%f",              current_error);
  d.addf("Max Current Error", "%f",          current_error_max);
  d.addf("Filtered Current Error", "%f",     abs_current_error);
  d.addf("Max Filtered Current Error", "%f", abs_current_error_max);

  d.addf("Motor Resistance Estimate", "%f", est_motor_resistance);
}


/**  \brief Call for each update. 
 *        
 *   Adds sample to motor model.  Also, adds sample to motor trace.  
 */
void MotorModel::sample(const ethercat_hardware::MotorTraceSample &s)
{

  const ethercat_hardware::ActuatorInfo &ai(actuator_info_);
  const ethercat_hardware::BoardInfo &bi(board_info_);
  
  // Estimate what voltage board should be outputting.
  double board_voltage = s.supply_voltage * s.programmed_pwm - bi.board_resistance * s.measured_current;
  
  // Compute motor voltage using motor model 
  double resistance_voltage = s.measured_current * ai.motor_resistance;
  double backemf_voltage =  s.velocity * ai.encoder_reduction * backemf_constant_;
  double motor_voltage =  resistance_voltage + backemf_voltage;

  // Compute limits for motor voltage error.
  const double resistance_error = 0.15;    // assume motor resistance can be off by 15%
  const double backemf_constant_error = 0.10; // assume backemf const can be off by 10%
  double motor_voltage_error_limit = 2.0 + fabs(resistance_voltage*resistance_error) + fabs(backemf_voltage * backemf_constant_error);
  // Put max limit on back emf voltage
  motor_voltage_error_limit = min(motor_voltage_error_limit, 10.0);

  // Estimate resitance
  double est_motor_resistance = 0.0;
  double est_motor_resistance_accuracy = 0.0;
  // Don't even try calculation if the is not enough motor current
  if (fabs(s.measured_current) > (0.02 * bi.hw_max_current + 0.005)) 
  {
    est_motor_resistance = (board_voltage - backemf_voltage) / s.measured_current;
    // not all resistance samples are created equal.  
    // When motor is not moving, result resistance calculation will be better,
    // because error in backemf constant value won't have any effect.
    est_motor_resistance_accuracy = 1.0 / (1.0 + fabs(backemf_voltage / resistance_voltage));
  }

  // Don't update filters if MCB is not enabled (halted)
  if (s.enabled) {
    filter_mutex_.lock();
    
    // Compare measured voltage to motor voltage.  Identify errors with broken inductor leads, etc
    measured_voltage_error_.sample(s.measured_motor_voltage - board_voltage);
    abs_measured_voltage_error_.sample( fabs(measured_voltage_error_.filter()) );
    
    // Compare motor and board voltage. Identify errors with motor or encoder
    motor_voltage_error_.sample((motor_voltage - board_voltage) / motor_voltage_error_limit);
    abs_motor_voltage_error_.sample( fabs(motor_voltage_error_.filter()) );
    
    // Compare measured/programmed only when board output voltage is not maxed out.   
    if ((s.programmed_pwm < bi.max_pwm_ratio*0.99) && (s.programmed_pwm > -bi.max_pwm_ratio*0.99))
    {
      current_error_.sample(s.measured_current-s.executed_current);
      abs_current_error_.sample( fabs(current_error_.filter()) );
    }
    
    abs_velocity_.sample(fabs(s.velocity));
    abs_board_voltage_.sample(fabs(board_voltage));
    abs_measured_current_.sample(fabs(s.measured_current));
    if (!trace_buffer_.empty()) {
      double position_delta = trace_buffer_.at(trace_index_).encoder_position - s.encoder_position;
      abs_position_delta_.sample(fabs(position_delta));
    }

    motor_resistance_.sample(est_motor_resistance, 0.005 * est_motor_resistance_accuracy);

    filter_mutex_.unlock();
  }

  { // Add motor trace sample to trace buffer
    assert(trace_buffer_.size() <= trace_size_);
    if (trace_buffer_.size() >= trace_size_) {
      trace_index_ = (trace_index_+1)%trace_buffer_.size();
      trace_buffer_.at(trace_index_) = s;
    } else {
      trace_index_ = trace_buffer_.size();
      trace_buffer_.push_back(s);
    }
  }

  // Add values calculated by model to new sample in trace
  {
    ethercat_hardware::MotorTraceSample &s2(trace_buffer_.at(trace_index_));
    s2.motor_voltage_error_limit           = motor_voltage_error_limit;
    s2.filtered_motor_voltage_error        = motor_voltage_error_.filter();
    s2.filtered_abs_motor_voltage_error    = abs_motor_voltage_error_.filter();
    s2.filtered_measured_voltage_error     = measured_voltage_error_.filter();
    s2.filtered_abs_measured_voltage_error = abs_measured_voltage_error_.filter();
    s2.filtered_current_error              = current_error_.filter();
    s2.filtered_abs_current_error          = abs_current_error_.filter();
  }

}


/** \brief Check for errors between sample data and motor model
 * 
 */
bool MotorModel::verify(std::string &reason) const
{
  bool rv=true;
  
  // Error limits should realy be parameters, not hardcoded.
  double measured_voltage_error_limit = board_info_.poor_measured_motor_voltage ? 10.0 : 4.0;
  double current_error_limit          = board_info_.hw_max_current * 0.3;

  bool is_measured_voltage_error = abs_measured_voltage_error_.filter() > measured_voltage_error_limit;
  bool is_motor_voltage_error    = abs_motor_voltage_error_.filter() > 1.0; // 1.0 = 100% motor_voltage_error_limit

  // Check back-EMF consistency
  if (is_motor_voltage_error || is_measured_voltage_error) 
  {
    reason = "Problem with the MCB, motor, encoder, or actuator model.";  
    rv = false;

    if( is_measured_voltage_error ) 
    {
      // Problem with board
      reason += " Board may be damaged.";
    }
    else if (is_motor_voltage_error) 
    {
      //Something is wrong with the encoder, the motor, or the motor board
      const double epsilon = 0.001;
      const double current_epsilon = 0.010;
      double encoder_tick_delta = 2 * M_PI / actuator_info_.pulses_per_revolution;

      //Try to diagnose further
      if ((abs_measured_current_.filter() < current_epsilon) && (abs_current_error_.filter() > current_epsilon))
      {
        //measured_current_ ~= 0 -> motor open-circuit likely
        reason += " Current near zero - check for unconnected motor leads.";
      }
      else if (abs_board_voltage_.filter() < epsilon)
      {
        //motor_voltage_ ~= 0 -> motor short-circuit likely
        reason += " Voltage near zero - check for short circuit.";
      }
      else if (abs_velocity_.filter() < epsilon)
      {
        // motor_velocity == 0 -> encoder failure likely
        reason += " Velocity near zero - check for encoder error.";
      }
      else if (abs_position_delta_.filter() < encoder_tick_delta)
      {
        // encoder changing by only 0 or 1 ticks --> check for disconnected wire.
        reason += " Encoder delta below 1 - check encoder wiring.";
      }
    }
  }
  else if (current_error_.filter() > current_error_limit)
  {
    //complain and shut down
    rv = false;
    reason = "Current loop error too large (MCB failing to hit desired current)";
  }

  return rv;
}

void MotorModel::SimpleFilter::sample(double value, double filter_coefficient)
{
  // F = A*V + (1-A)*F 
  // F = A*V + F - A*F 
  // F = F + A*(V-F)
  filtered_value_ += filter_coefficient * (value-filtered_value_);  
}

void MotorModel::Filter::sample(double value)
{
  SimpleFilter::sample(value, filter_coefficient_);
  max_filtered_value_ = max(fabs(filtered_value_), max_filtered_value_);
}

MotorModel::SimpleFilter::SimpleFilter()
{
  reset();
}

MotorModel::Filter::Filter(double filter_coefficient) :
  SimpleFilter(),
  filter_coefficient_(filter_coefficient)
{
  reset();
}

void MotorModel::SimpleFilter::reset()
{
  filtered_value_ = 0.0;
}

void MotorModel::Filter::reset()
{ 
  SimpleFilter::reset();
  max_filtered_value_ = 0.0;
}
