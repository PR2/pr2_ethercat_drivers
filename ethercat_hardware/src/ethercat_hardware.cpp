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

#include "ethercat_hardware/ethercat_hardware.h"

#include <ethercat/ethercat_xenomai_drv.h>
#include <dll/ethercat_dll.h>
#include <dll/ethercat_device_addressed_telegram.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <boost/foreach.hpp>

EthercatHardwareDiagnostics::EthercatHardwareDiagnostics() :

  txandrx_errors_(0),
  device_count_(0),
  pd_error_(false),
  halt_after_reset_(false),
  reset_motors_service_count_(0), 
  halt_motors_service_count_(0),
  halt_motors_error_count_(0)
{

}

void EthercatHardwareDiagnostics::resetMaxTiming()
{
  max_pack_command_  = 0.0;
  max_txandrx_       = 0.0;
  max_unpack_state_  = 0.0;
  max_publish_       = 0.0;
}

EthercatHardware::EthercatHardware(const std::string& name) :
  hw_(0), ni_(0), this_buffer_(0), prev_buffer_(0), buffer_size_(0), halt_motors_(true), reset_state_(0), 
  diagnostics_publisher_(name), 
  motor_publisher_(ros::NodeHandle(name), "motors_halted", 1, true), 
  device_loader_("ethercat_hardware", "EthercatDevice")
{

}

EthercatHardware::~EthercatHardware()
{
  diagnostics_publisher_.stop();
  if (slaves_)
  {
    for (uint32_t i = 0; i < num_slaves_; ++i)
    {
      EC_FixedStationAddress fsa(i + 1);
      EtherCAT_SlaveHandler *sh = em_->get_slave_handler(fsa);
      if (sh) sh->to_state(EC_PREOP_STATE);
      delete slaves_[i];
    }
    delete[] slaves_;
  }
  if (ni_)
  {
    close_socket(ni_);
  }
  delete[] buffers_;
  delete hw_;
  motor_publisher_.stop();
}


void EthercatHardware::changeState(EtherCAT_SlaveHandler *sh, EC_State new_state)
{
  unsigned product_code = sh->get_product_code();
  unsigned serial = sh->get_serial();
  uint32_t revision = sh->get_revision();
  unsigned slave = sh->get_station_address()-1;

  if (!sh->to_state(new_state))
  {
    ROS_FATAL("Cannot goto state %d for slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
              new_state, slave, product_code, product_code, serial, serial, revision, revision);
    if ((product_code==0xbaddbadd) || (serial==0xbaddbadd) || (revision==0xbaddbadd))
      ROS_FATAL("Note: 0xBADDBADD indicates that the value was not read correctly from device.");
    ROS_BREAK();
  }
}

void EthercatHardware::init(char *interface, bool allow_unprogrammed)
{
  // open temporary socket to use with ioctl
  int sock = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    int error = errno;
    ROS_FATAL("Couldn't open temp socket : %s", strerror(error));
    sleep(1);
    ROS_BREAK();    
  }
  
  struct ifreq ifr;
  strncpy(ifr.ifr_name, interface, IFNAMSIZ);
  if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
    int error = errno;
    ROS_FATAL("Cannot get interface flags for %s: %s", interface, strerror(error));
    sleep(1);
    ROS_BREAK();
  }

  close(sock);
  sock = -1;

  if (!(ifr.ifr_flags & IFF_UP)) {
    ROS_FATAL("Interface %s is not UP. Try : ifup %s", interface, interface);
    sleep(1);
    ROS_BREAK();
  }
  if (!(ifr.ifr_flags & IFF_RUNNING)) {
    ROS_FATAL("Interface %s is not RUNNING. Is cable plugged in and device powered?", interface);
    sleep(1);
    ROS_BREAK();
  }


  // Initialize network interface
  interface_ = interface;
  if ((ni_ = init_ec(interface)) == NULL)
  {
    ROS_FATAL("Unable to initialize interface: %s", interface);
    sleep(1);
    ROS_BREAK();
  }

  oob_com_ = new EthercatOobCom(ni_);

  // Initialize Application Layer (AL)
  EtherCAT_DataLinkLayer::instance()->attach(ni_);
  if ((al_ = EtherCAT_AL::instance()) == NULL)
  {
    ROS_FATAL("Unable to initialize Application Layer (AL): %p", al_);
    sleep(1);
    ROS_BREAK();
  }

  num_slaves_ = al_->get_num_slaves();
  if (num_slaves_ == 0)
  {
    ROS_FATAL("Unable to locate any slaves");
    sleep(1);
    ROS_BREAK();
  }

  // Initialize Master
  if ((em_ = EtherCAT_Master::instance()) == NULL)
  {
    ROS_FATAL("Unable to initialize EtherCAT_Master: %p", em_);
    sleep(1);
    ROS_BREAK();
  }

  slaves_ = new EthercatDevice*[num_slaves_];

  // Make temporary list of slave handles
  std::vector<EtherCAT_SlaveHandler*> slave_handles;
  for (unsigned int slave = 0; slave < num_slaves_; ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = em_->get_slave_handler(fsa);
    if (sh == NULL)
    {
      ROS_FATAL("Unable to get slave handler #%d", slave);
      sleep(1);
      ROS_BREAK();
    }
    slave_handles.push_back(sh);
  }

  // Configure slaves
  BOOST_FOREACH(EtherCAT_SlaveHandler *sh, slave_handles)
  {
    unsigned product_code = sh->get_product_code();
    unsigned serial = sh->get_serial();
    uint32_t revision = sh->get_revision();
    unsigned slave = sh->get_station_address()-1;
    
    if ((slaves_[slave] = configSlave(sh)) == NULL)
    {
      ROS_FATAL("Unable to configure slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
                slave, product_code, product_code, serial, serial, revision, revision);
      if ((product_code==0xbaddbadd) || (serial==0xbaddbadd) || (revision==0xbaddbadd))
        ROS_FATAL("Note: 0xBADDBADD indicates that the value was not read correctly from device.");
      ROS_FATAL("Perhaps you should power-cycle the MCBs");
      sleep(1);
      ROS_BREAK();
    }
    buffer_size_ += slaves_[slave]->command_size_ + slaves_[slave]->status_size_;
  }

  // Move slave from INIT to PREOP
  BOOST_FOREACH(EtherCAT_SlaveHandler *sh, slave_handles)
  {
    changeState(sh,EC_PREOP_STATE);
  }

  // Move slave from PREOP to SAFEOP
  BOOST_FOREACH(EtherCAT_SlaveHandler *sh, slave_handles)
  {
    changeState(sh,EC_SAFEOP_STATE);
  }

  // Move slave from SAFEOP to OP
  // TODO : move to OP after initializing slave process data
  BOOST_FOREACH(EtherCAT_SlaveHandler *sh, slave_handles)
  {
    changeState(sh,EC_OP_STATE);
  }

  // Allocate buffers to send and receive commands
  buffers_ = new unsigned char[2 * buffer_size_];
  this_buffer_ = buffers_;
  prev_buffer_ = buffers_ + buffer_size_;

  // Make sure motors are disabled, also collect status data
  memset(this_buffer_, 0, 2 * buffer_size_);
  if (!txandrx_PD(buffer_size_, this_buffer_, 20))
  {
    ROS_FATAL("No communication with devices");
    ROS_BREAK();
  }
  
  // prev_buffer should contain valid status data when update function is first used
  memcpy(prev_buffer_, this_buffer_, buffer_size_);

  // Create pr2_hardware_interface::HardwareInterface
  hw_ = new pr2_hardware_interface::HardwareInterface();
  hw_->current_time_ = ros::Time::now();
  last_published_ = hw_->current_time_;
  motor_last_published_ = hw_->current_time_;

  // Initialize slaves
  //set<string> actuator_names;
  for (unsigned int slave = 0; slave < num_slaves_; ++slave)
  {
    if (slaves_[slave]->initialize(hw_, allow_unprogrammed) < 0)
    {
      EtherCAT_SlaveHandler *sh = slaves_[slave]->sh_;
      ROS_FATAL("Unable to initialize slave #%d, , product code: %d, revision: %d, serial: %d",
                slave, sh->get_product_code(), sh->get_revision(), sh->get_serial());
      sleep(1);
      ROS_BREAK();
    }
  }

  diagnostics_publisher_.initialize(interface_, buffer_size_, slaves_, num_slaves_);
}


EthercatHardwareDiagnosticsPublisher::EthercatHardwareDiagnosticsPublisher(const std::string &name) :
  diagnostics_ready_(false),
  publisher_(ros::NodeHandle(name), "/diagnostics", 1),
  diagnostics_buffer_(NULL),
  slaves_(NULL),
  num_slaves_(0)
{
  diagnostics_thread_ = boost::thread(boost::bind(&EthercatHardwareDiagnosticsPublisher::diagnosticsThreadFunc, this));
}

EthercatHardwareDiagnosticsPublisher::~EthercatHardwareDiagnosticsPublisher()
{
  delete[] diagnostics_buffer_;
}

void EthercatHardwareDiagnosticsPublisher::initialize(const string &interface, unsigned int buffer_size, EthercatDevice **slaves, unsigned int num_slaves)
{
  interface_ = interface;
  buffer_size_ = buffer_size;
  slaves_ = slaves;
  num_slaves_ = num_slaves;

  diagnostics_buffer_ = new unsigned char[buffer_size_];

  // Initialize diagnostic data structures
  publisher_.msg_.status.reserve(num_slaves_ + 1);
  statuses_.reserve(num_slaves_ + 1);
  values_.reserve(10);
}

void EthercatHardwareDiagnosticsPublisher::publish(
         const unsigned char *buffer, 
         const EthercatHardwareDiagnostics &diagnostics)
{
  boost::try_to_lock_t try_lock;
  boost::unique_lock<boost::mutex> lock(diagnostics_mutex_, try_lock);
  if (lock.owns_lock())
  {
    // Make copies of diagnostic data for dianostic thread
    memcpy(diagnostics_buffer_, buffer, buffer_size_);
    diagnostics_ = diagnostics;
    // Trigger diagnostics publish thread
    diagnostics_ready_ = true;
    diagnostics_cond_.notify_one();
  }
}

void EthercatHardwareDiagnosticsPublisher::stop()
{
  diagnostics_thread_.interrupt();
  diagnostics_thread_.join();
  publisher_.stop();
}

void EthercatHardwareDiagnosticsPublisher::diagnosticsThreadFunc()
{
  try {
    while (1) {
      boost::unique_lock<boost::mutex> lock(diagnostics_mutex_);
      while (!diagnostics_ready_) {
        diagnostics_cond_.wait(lock);
      }
      diagnostics_ready_ = false;
      publishDiagnostics();
    }
  } catch (boost::thread_interrupted const&) {
    return;
  }
}

void EthercatHardwareDiagnosticsPublisher::timingInformation(
        diagnostic_updater::DiagnosticStatusWrapper &status, 
        const string &key, 
        const accumulator_set<double, stats<tag::max, tag::mean> > &acc,
        double max)
{
  status.addf(key + " Avg (us)",       "%5.4f", extract_result<tag::mean>(acc) * 1e6); // Average over last 1 second
  status.addf(key + " 1 Sec Max (us)", "%5.4f", extract_result<tag::max>(acc) * 1e6);  // Max over last 1 second
  status.addf(key + " Max (us)",       "%5.4f", max * 1e6);                            // Max since start
}

void EthercatHardwareDiagnosticsPublisher::publishDiagnostics()
{  
  // Publish status of EtherCAT master
  status_.clearSummary();
  status_.clear();    

  status_.name = "EtherCAT Master";
  if (diagnostics_.motors_halted_)
  {
    if (diagnostics_.halt_after_reset_) 
    {
      status_.summary(status_.ERROR, "Motors halted soon after reset");
    }
    else
    {
      status_.summary(status_.ERROR, "Motors halted");
    }
  } else {
    status_.summary(status_.OK, "OK");
  }

  if (diagnostics_.pd_error_)
  {
    status_.mergeSummary(status_.ERROR, "Error sending proccess data");
  }

  status_.add("Motors halted", diagnostics_.motors_halted_ ? "true" : "false");
  status_.addf("EtherCAT devices (expected)", "%d", num_slaves_); 
  status_.addf("EtherCAT devices (current)",  "%d", diagnostics_.device_count_); 
  status_.add("Interface", interface_);
  //status_.addf("Reset state", "%d", reset_state_);

  // Produce warning if number of devices changed after device initalization
  if (num_slaves_ != diagnostics_.device_count_) {
    status_.mergeSummary(status_.WARN, "Number of EtherCAT devices changed");
  }

  timingInformation(status_, "Roundtrip time", diagnostics_.txandrx_acc_, diagnostics_.max_txandrx_);
  if (diagnostics_.collect_extra_timing_)
  {
    timingInformation(status_, "Pack command time", diagnostics_.pack_command_acc_, diagnostics_.max_pack_command_);
    timingInformation(status_, "Unpack state time", diagnostics_.unpack_state_acc_, diagnostics_.max_unpack_state_);
    timingInformation(status_, "Publish time", diagnostics_.publish_acc_, diagnostics_.max_publish_);
  }

  status_.addf("EtherCAT Process Data txandrx errors", "%d", diagnostics_.txandrx_errors_);

  status_.addf("Reset motors service count", "%d", diagnostics_.reset_motors_service_count_);
  status_.addf("Halt motors service count", "%d", diagnostics_.halt_motors_service_count_);
  status_.addf("Halt motors error count", "%d", diagnostics_.halt_motors_error_count_);

  { // Publish ethercat network interface counters 
    const struct netif_counters *c = &diagnostics_.counters_;
    status_.add("Input Thread",       (diagnostics_.input_thread_is_stopped_ ? "Stopped" : "Running"));
    status_.addf("Sent Packets",        "%lld", c->sent);
    status_.addf("Received Packets",    "%lld", c->received);
    status_.addf("Collected Packets",   "%lld", c->collected);
    status_.addf("Dropped Packets",     "%lld", c->dropped);
    status_.addf("TX Errors",           "%lld", c->tx_error);
    status_.addf("TX Network Down",     "%lld", c->tx_net_down);
    status_.addf("TX Queue Full",       "%lld", c->tx_full);
    status_.addf("RX Runt Packet",      "%lld", c->rx_runt_pkt);
    status_.addf("RX Not EtherCAT",     "%lld", c->rx_not_ecat);
    status_.addf("RX Other EML",        "%lld", c->rx_other_eml);
    status_.addf("RX Bad Index",        "%lld", c->rx_bad_index);
    status_.addf("RX Bad Sequence",     "%lld", c->rx_bad_seqnum);
    status_.addf("RX Duplicate Sequence", "%lld", c->rx_dup_seqnum);    
    status_.addf("RX Duplicate Packet", "%lld", c->rx_dup_pkt);    
    status_.addf("RX Bad Order",        "%lld", c->rx_bad_order);    
    status_.addf("RX Late Packet",      "%lld", c->rx_late_pkt);
    status_.addf("RX Late Packet RTT",  "%lld", c->rx_late_pkt_rtt_us);
    
    double rx_late_pkt_rtt_us_avg = 0.0;
    if (c->rx_late_pkt > 0) {
      rx_late_pkt_rtt_us_avg = ((double)c->rx_late_pkt_rtt_us_sum)/((double)c->rx_late_pkt);
    }
    status_.addf("RX Late Packet Avg RTT", "%f", rx_late_pkt_rtt_us_avg);
  }

  statuses_.clear();
  statuses_.push_back(status_);

  // Also, collect diagnostic statuses of all EtherCAT device
  unsigned char *current = diagnostics_buffer_;
  for (unsigned int s = 0; s < num_slaves_; ++s)
  {
    slaves_[s]->multiDiagnostics(statuses_, current);
    current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
  }

  // Publish status of each EtherCAT device
  if (publisher_.trylock())
  {
    publisher_.msg_.set_status_vec(statuses_);
    publisher_.msg_.header.stamp = ros::Time::now();
    publisher_.unlockAndPublish();
  }
}

void EthercatHardware::update(bool reset, bool halt)
{
  // Update current time
  ros::Time update_start_time(ros::Time::now());
  hw_->current_time_ = update_start_time;

  unsigned char *this_buffer, *prev_buffer;
  bool old_halt_motors = halt_motors_;
  bool halt_motors_error = false;  // True if motors halted due to error

  // Convert HW Interface commands to MCB-specific buffers
  this_buffer = this_buffer_;

  if (halt)
  {
    ++diagnostics_.halt_motors_service_count_;
    halt_motors_ = true;
  }

  // Resetting devices should clear device errors and release devices from halt.
  // To reduce load on power system, release devices from halt, one at a time 
  const unsigned CYCLES_PER_HALT_RELEASE = 2; // Wait two cycles between releasing each device
  if (reset)
  {
    ++diagnostics_.reset_motors_service_count_;
    reset_state_ = CYCLES_PER_HALT_RELEASE * num_slaves_ + 5;
    last_reset_ = update_start_time;
    diagnostics_.halt_after_reset_ = false;
  }
  bool reset_devices = reset_state_ == CYCLES_PER_HALT_RELEASE * num_slaves_ + 3;
  if (reset_devices)
  {
    halt_motors_ = false;
    diagnostics_.resetMaxTiming();
    diagnostics_.pd_error_ = false;
  }

  for (unsigned int s = 0; s < num_slaves_; ++s)
  {
    // Pack the command structures into the EtherCAT buffer
    // Disable the motor if they are halted or coming out of reset
    bool halt_device = halt_motors_ || ((s*CYCLES_PER_HALT_RELEASE+1) < reset_state_);
    slaves_[s]->packCommand(this_buffer, halt_device, reset_devices);
    this_buffer += slaves_[s]->command_size_ + slaves_[s]->status_size_;
  }

  // Transmit process data
  ros::Time txandrx_start_time(ros::Time::now()); // Also end time for pack_command_stage
  diagnostics_.pack_command_acc_((txandrx_start_time-update_start_time).toSec());

  // Send/receive device proccess data
  bool success = txandrx_PD(buffer_size_, this_buffer_, 5);

  ros::Time txandrx_end_time(ros::Time::now());  // Also begining of unpack_state 
  diagnostics_.txandrx_acc_((txandrx_end_time - txandrx_start_time).toSec());

  if (!success)
  {
    // If process data didn't get sent after multiple retries, stop motors
    halt_motors_error = true;
    halt_motors_ = true;    
    diagnostics_.pd_error_ = true;
  }
  else
  {
    // Convert status back to HW Interface
    this_buffer = this_buffer_;
    prev_buffer = prev_buffer_;
    for (unsigned int s = 0; s < num_slaves_; ++s)
    {
      if (!slaves_[s]->unpackState(this_buffer, prev_buffer) && !reset_devices)
      {
        halt_motors_error = true;
        halt_motors_ = true;
      }
      this_buffer += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      prev_buffer += slaves_[s]->command_size_ + slaves_[s]->status_size_;
    }
    
    if (reset_state_)
      --reset_state_;
    
    unsigned char *tmp = this_buffer_;
    this_buffer_ = prev_buffer_;
    prev_buffer_ = tmp;
  }

  ros::Time unpack_end_time;
  if (diagnostics_.collect_extra_timing_)
  {
    unpack_end_time = ros::Time::now();  // also start of publish time                            
    diagnostics_.unpack_state_acc_((unpack_end_time - txandrx_end_time).toSec());
  }

  if ((update_start_time - last_published_) > ros::Duration(1.0))
  {
    last_published_ = update_start_time;
    publishDiagnostics();
  }

  if (halt_motors_ != old_halt_motors ||
      (update_start_time - motor_last_published_) > ros::Duration(1.0))
  {
    motor_last_published_ = update_start_time;
    motor_publisher_.lock();
    motor_publisher_.msg_.data = halt_motors_;
    motor_publisher_.unlockAndPublish();
  }

  if ((halt_motors_ && !old_halt_motors) && (halt_motors_error))
  {
    ++diagnostics_.halt_motors_error_count_;
    if ((update_start_time - last_reset_) < ros::Duration(0.5))
    {
      diagnostics_.halt_after_reset_ = true;
    }
  }    

  if (diagnostics_.collect_extra_timing_)
  {
    ros::Time publish_end_time(ros::Time::now());  
    diagnostics_.publish_acc_((publish_end_time - unpack_end_time).toSec());
  }
}

void EthercatHardware::updateAccMax(double &max, const accumulator_set<double, stats<tag::max, tag::mean> > &acc)
{
  max = std::max(max, extract_result<tag::max>(acc));
}

void EthercatHardware::publishDiagnostics()
{
  // Update max timing values
  updateAccMax(diagnostics_.max_pack_command_, diagnostics_.pack_command_acc_);
  updateAccMax(diagnostics_.max_txandrx_,      diagnostics_.txandrx_acc_);
  updateAccMax(diagnostics_.max_unpack_state_, diagnostics_.unpack_state_acc_);
  updateAccMax(diagnostics_.max_publish_,      diagnostics_.publish_acc_);

  // Grab stats and counters from input thread
  diagnostics_.counters_ = ni_->counters;
  diagnostics_.input_thread_is_stopped_ = bool(ni_->is_stopped);

  diagnostics_.motors_halted_ = halt_motors_;

  // Pass diagnostic data to publisher thread
  diagnostics_publisher_.publish(this_buffer_, diagnostics_);

  // Clear statistics accumulators
  static accumulator_set<double, stats<tag::max, tag::mean> > blank;
  diagnostics_.pack_command_acc_ = blank;
  diagnostics_.txandrx_acc_      = blank;
  diagnostics_.unpack_state_acc_ = blank;
  diagnostics_.publish_acc_      = blank;
}

EthercatDevice *
EthercatHardware::configSlave(EtherCAT_SlaveHandler *sh)
{
  static int start_address = 0x00010000;

  EthercatDevice *p = NULL;
  stringstream str;
  str << sh->get_product_code();
  try {
    p = device_loader_.createClassInstance(str.str());
  }
  catch (pluginlib::LibraryLoadException &e)
  {
    p = NULL;
  }
  if (p) {
    p->construct(sh, start_address);
  }
  return p;
}


void EthercatHardware::collectDiagnostics()
{
  if (NULL == oob_com_)
    return;

  { // Count number of devices 
    EC_Logic *logic = EC_Logic::instance();
    unsigned char p[1];
    EC_UINT length = sizeof(p);
    
    // Build read telegram, use slave position
    APRD_Telegram status(logic->get_idx(), // Index
                         0, // Slave position on ethercat chain (auto increment address)
                         0, // ESC physical memory address (start address)
                         logic->get_wkc(), // Working counter
                         length, // Data Length,
                         p); // Buffer to put read result into
    
    // Put read telegram in ethercat/ethernet frame
    EC_Ethernet_Frame frame(&status);    
    oob_com_->txandrx(&frame);

    // Worry about locking for single value?
    diagnostics_.device_count_ = status.get_adp();
  }

  for (unsigned i = 0; i < num_slaves_; ++i)
  {    
    EthercatDevice * d(slaves_[i]);
    d->collectDiagnostics(oob_com_);
  }
}


// Prints (error) counter infomation of network interface driver
void EthercatHardware::printCounters(std::ostream &os) 
{  
  const struct netif_counters &c(ni_->counters);      
  os << "netif counters :" << endl
     << " sent          = " << c.sent << endl
     << " received      = " << c.received << endl
     << " collected     = " << c.collected << endl
     << " dropped       = " << c.dropped << endl
     << " tx_error      = " << c.tx_error << endl
     << " tx_net_down   = " << c.tx_net_down << endl
     << " tx_full       = " << c.tx_full << endl
     << " rx_runt_pkt   = " << c.rx_runt_pkt << endl
     << " rx_not_ecat   = " << c.rx_not_ecat << endl
     << " rx_other_eml  = " << c.rx_other_eml << endl
     << " rx_bad_index  = " << c.rx_bad_index << endl
     << " rx_bad_seqnum = " << c.rx_bad_seqnum << endl
     << " rx_dup_seqnum = " << c.rx_dup_seqnum << endl
     << " rx_dup_pkt    = " << c.rx_dup_pkt << endl
     << " rx_bad_order  = " << c.rx_bad_order << endl
     << " rx_late_pkt   = " << c.rx_late_pkt << endl;
}


bool EthercatHardware::txandrx_PD(unsigned buffer_size, unsigned char* buffer, unsigned tries)
{
  // Try multiple times to get proccess data to device
  bool success = false;
  for (unsigned i=0; i<tries && !success; ++i) {
    // Try transmitting process data
    success = em_->txandrx_PD(buffer_size_, this_buffer_);
    if (!success) {
      ++diagnostics_.txandrx_errors_;
    } 
    // Transmit new OOB data
    oob_com_->tx();
  }
  return success;
}


bool EthercatHardware::publishTrace(int position, const string &reason, unsigned level, unsigned delay)
{
  if (position >= (int)num_slaves_)
  {
    ROS_WARN("Invalid device position %d.  Use 0-%d, or -1.", position, num_slaves_-1);    
    return false;
  }

  if (level > 2) 
  {
    ROS_WARN("Invalid level : %d.  Using level=2 (ERROR).", level);
    level = 2;
  }

  string new_reason("Manually triggered : " + reason);
  
  bool retval = false;
  if (position < 0) 
  {
    for (unsigned i=0; i<num_slaves_; ++i)
    {
      if (slaves_[i]->publishTrace(new_reason,level,delay))
      {
        retval = true;
      }
    }
  }
  else 
  {
    retval = slaves_[position]->publishTrace(new_reason,level,delay);
    if (!retval) 
    {
      ROS_WARN("Device %d does not support publishing trace", position);
    }
  }
  return retval;
}
