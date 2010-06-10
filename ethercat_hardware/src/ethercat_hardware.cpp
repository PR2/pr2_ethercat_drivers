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

EthercatHardware::EthercatHardware(const std::string& name) :
  hw_(0), ni_(0), this_buffer_(0), prev_buffer_(0), diagnostics_buffer_(0), buffer_size_(0), halt_motors_(true), reset_state_(0), motor_publisher_(ros::NodeHandle(name), "motors_halted", 1, true), publisher_(ros::NodeHandle(name), "/diagnostics", 1), device_loader_("ethercat_hardware", "EthercatDevice")
{
  diagnostics_.max_roundtrip_ = 0;
  diagnostics_.txandrx_errors_ = 0;
  diagnostics_.device_count_ = 0;
  diagnostics_.pd_error_ = false;
  diagnostics_ready_ = false;
  diagnostics_thread_ = boost::thread(boost::bind(&EthercatHardware::diagnosticsThreadFunc, this));
}

EthercatHardware::~EthercatHardware()
{
  diagnostics_thread_.interrupt();
  diagnostics_thread_.join();
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
  delete[] diagnostics_buffer_;
  delete hw_;
  motor_publisher_.stop();
  publisher_.stop();
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

  // Configure slaves
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

    unsigned product_code = sh->get_product_code();
    unsigned serial = sh->get_serial();
    uint32_t revision = sh->get_revision();

    if ((slaves_[slave] = configSlave(sh)) != NULL)
    {
      if (!sh->to_state(EC_OP_STATE))
      {
        ROS_FATAL("Cannot goto OP state for slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
                  slave, product_code, product_code, serial, serial, revision, revision);
        if ((product_code==0xbaddbadd) || (serial==0xbaddbadd) || (revision==0xbaddbadd))
          ROS_FATAL("Note: 0xBADDBADD indicates that the value was not read correctly from device.");
        ROS_BREAK();
      }
      buffer_size_ += slaves_[slave]->command_size_ + slaves_[slave]->status_size_;
    }
    else
    {
      uint32_t product_code = sh->get_product_code();
      ROS_FATAL("Unable to configure slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
                slave, product_code, product_code, serial, serial, revision, revision);
      if ((product_code==0xbaddbadd) || (serial==0xbaddbadd) || (revision==0xbaddbadd))
        ROS_FATAL("Note: 0xBADDBADD indicates that the value was not read correctly from device.");
      ROS_FATAL("Perhaps you should power-cycle the MCBs");
      sleep(1);
      ROS_BREAK();
    }
  }

  // Allocate buffers to send and receive commands
  buffers_ = new unsigned char[2 * buffer_size_];
  this_buffer_ = buffers_;
  prev_buffer_ = buffers_ + buffer_size_;
  diagnostics_buffer_ = new unsigned char[buffer_size_];

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

  // Initialize diagnostic data structures
  publisher_.msg_.status.reserve(num_slaves_ + 1);
  statuses_.reserve(num_slaves_ + 1);
  values_.reserve(10);
}

void EthercatHardware::diagnosticsThreadFunc()
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

void EthercatHardware::publishDiagnostics()
{
  // Publish status of EtherCAT master
  diagnostic_updater::DiagnosticStatusWrapper status;
  
  statuses_.clear();

  status.name = "EtherCAT Master";
  if (halt_motors_)
  {
    status.summary(status.ERROR, "Motors halted");
  } else {
    status.summary(status.OK, "OK");
  }

  if (diagnostics_.pd_error_)
  {
    status.mergeSummary(status.ERROR, "Error sending proccess data");
  }

  status.add("Motors halted", halt_motors_ ? "true" : "false");
  status.addf("EtherCAT devices (expected)", "%d", num_slaves_); 
  status.addf("EtherCAT devices (current)",  "%d", diagnostics_.device_count_); 
  status.add("Interface", interface_);
  status.addf("Reset state", "%d", reset_state_);

  // Produce warning if number of devices changed after device initalization
  if (num_slaves_ != diagnostics_.device_count_) {
    status.mergeSummary(status.WARN, "Number of EtherCAT devices changed");
  }

  // Roundtrip
  diagnostics_.max_roundtrip_ = std::max(diagnostics_.max_roundtrip_, 
       extract_result<tag::max>(diagnostics_.acc_));
  status.addf("Average roundtrip time (us)", "%.4f", extract_result<tag::mean>(diagnostics_.acc_) * 1e6);

  accumulator_set<double, stats<tag::max, tag::mean> > blank;
  diagnostics_.acc_ = blank;

  status.addf("Maximum roundtrip time (us)", "%.4f", diagnostics_.max_roundtrip_ * 1e6);
  status.addf("EtherCAT Process Data txandrx errors", "%d", diagnostics_.txandrx_errors_);

  { // Publish ethercat network interface counters 
    const struct netif_counters *c = &ni_->counters;
    status.add("Input Thread",       ((ni_->is_stopped!=0) ? "Stopped" : "Running"));
    status.addf("Sent Packets",        "%lld", c->sent);
    status.addf("Received Packets",    "%lld", c->received);
    status.addf("Collected Packets",   "%lld", c->collected);
    status.addf("Dropped Packets",     "%lld", c->dropped);
    status.addf("TX Errors",           "%lld", c->tx_error);
    status.addf("TX Network Down",     "%lld", c->tx_net_down);
    status.addf("TX Queue Full",       "%lld", c->tx_full);
    status.addf("RX Runt Packet",      "%lld", c->rx_runt_pkt);
    status.addf("RX Not EtherCAT",     "%lld", c->rx_not_ecat);
    status.addf("RX Other EML",        "%lld", c->rx_other_eml);
    status.addf("RX Bad Index",        "%lld", c->rx_bad_index);
    status.addf("RX Bad Sequence",     "%lld", c->rx_bad_seqnum);
    status.addf("RX Duplicate Sequence", "%lld", c->rx_dup_seqnum);    
    status.addf("RX Duplicate Packet", "%lld", c->rx_dup_pkt);    
    status.addf("RX Bad Order",        "%lld", c->rx_bad_order);    
    status.addf("RX Late Packet",      "%lld", c->rx_late_pkt);
    status.addf("RX Late Packet RTT",  "%lld", c->rx_late_pkt_rtt_us);
    
    double rx_late_pkt_rtt_us_avg = 0.0;
    if (c->rx_late_pkt > 0) {
      rx_late_pkt_rtt_us_avg = ((double)c->rx_late_pkt_rtt_us_sum)/((double)c->rx_late_pkt);
    }
    status.addf("RX Late Packet Avg RTT", "%f", rx_late_pkt_rtt_us_avg);
  }

  statuses_.push_back(status);

  unsigned char *current = diagnostics_buffer_;
  for (unsigned int s = 0; s < num_slaves_; ++s)
  {
    slaves_[s]->diagnostics(status, current);
    statuses_.push_back(status);
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
  unsigned char *this_buffer, *prev_buffer;
  bool old_halt_motors = halt_motors_;

  // Update current time
  hw_->current_time_ = ros::Time::now();

  // Convert HW Interface commands to MCB-specific buffers
  this_buffer = this_buffer_;

  if (halt)
    halt_motors_ = true;

  // Resetting devices should clear device errors and release devices from halt.
  // To reduce load on power system, release devices from halt, one at a time 
  const unsigned CYCLES_PER_HALT_RELEASE = 2; // Wait two cycles between releasing each device
  if (reset)
  {
    reset_state_ = CYCLES_PER_HALT_RELEASE * num_slaves_ + 5;
  }
  bool reset_devices = reset_state_ == CYCLES_PER_HALT_RELEASE * num_slaves_ + 3;
  if (reset_devices)
  {
    halt_motors_ = false;
    diagnostics_.max_roundtrip_ = 0;
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
  ros::Time start = ros::Time::now();

  // Send/receive device proccess data
  bool success = txandrx_PD(buffer_size_, this_buffer_, 5);

  diagnostics_.acc_((ros::Time::now() - start).toSec());

  if (!success)
  {
    // If process data didn't get sent after multiple retries, stop motors
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

  if ((hw_->current_time_ - last_published_) > ros::Duration(1.0))
  {
    last_published_ = hw_->current_time_;
    memcpy(diagnostics_buffer_, this_buffer_, buffer_size_);
    diagnostics_ready_ = true;
    diagnostics_cond_.notify_one();
  }

  if (halt_motors_ != old_halt_motors ||
      (hw_->current_time_ - motor_last_published_) > ros::Duration(1.0))
  {
    motor_last_published_ = hw_->current_time_;
    motor_publisher_.lock();
    motor_publisher_.msg_.data = halt_motors_;
    motor_publisher_.unlockAndPublish();
  }
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
