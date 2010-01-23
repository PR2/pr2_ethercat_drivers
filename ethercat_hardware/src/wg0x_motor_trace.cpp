
#include <ethercat_hardware/wg0x_motor_trace.h>

WG0XMotorTrace::WG0XMotorTrace(unsigned max_size) : max_size_(max_size), index_(0)
{
  assert(max_size_ > 0);
  trace_buffer_.reserve(max_size_);
}

/**  \brief Initializes motor trace publisher
 */
bool WG0XMotorTrace::initialize(const ethercat_hardware::ActuatorInfo &actuator_info, const ethercat_hardware::BoardInfo &board_info)
{
  std::string topic("motor_trace");
  if (!actuator_info.name.empty())
    topic = topic + "/" + actuator_info.name;
  publisher_ = new realtime_tools::RealtimePublisher<ethercat_hardware::MotorTrace>(ros::NodeHandle(), topic, 1, true);
  if (publisher_ == NULL) 
    return false;
  
  {
   ethercat_hardware::MotorTrace &msg(publisher_->msg_);
   msg.actuator_info = actuator_info;  
   msg.board_info = board_info;
   msg.samples.reserve(max_size_);
  }
  
  return true;
}

/**  \brief Publishes motor trace
 */
void WG0XMotorTrace::publish()
{
  assert(publisher_ != NULL);
  if ((publisher_==NULL) || (!publisher_->trylock())) 
    return;
  
  ethercat_hardware::MotorTrace &msg(publisher_->msg_);
  
  msg.header.stamp = ros::Time::now();  
  unsigned size=trace_buffer_.size();
  msg.samples.clear();
  msg.samples.reserve(size);

  // TODO : is there a beter way to copy data between two std::vectors
  for (unsigned i=0; i<size; ++i) {
    msg.samples.push_back(trace_buffer_.at((index_+1+i)%size));
  }

  publisher_->unlockAndPublish();
}

/**  \brief Adds sample to trace buffer
 */
void WG0XMotorTrace::add_sample(const ethercat_hardware::MotorTraceSample &s)
{
  assert(trace_buffer_.size() <= max_size_);

  if (trace_buffer_.size() >= max_size_) {
    index_ = (index_+1)%trace_buffer_.size();
    trace_buffer_.at(index_) = s;
  } else {
    index_ = trace_buffer_.size();
    trace_buffer_.push_back(s);
  }
}
