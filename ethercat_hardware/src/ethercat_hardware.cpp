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

EthercatHardware::EthercatHardware() :
  hw_(0), ni_(0), current_buffer_(0), last_buffer_(0), buffer_size_(0), halt_motors_(true), reset_state_(0), publisher_(ros::NodeHandle(), "/diagnostics", 1)
{
  diagnostics_.max_roundtrip_ = 0;
  diagnostics_.txandrx_errors_ = 0;
}

EthercatHardware::~EthercatHardware()
{
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
  if (buffers_)
  {
    delete[] buffers_;
  }
  if (hw_)
  {
    delete hw_;
  }
  publisher_.stop();
}

void EthercatHardware::init(char *interface, bool allow_unprogrammed)
{
  // Initialize network interface
  interface_ = interface;
  if ((ni_ = init_ec(interface)) == NULL)
  {
    ROS_FATAL("Unable to initialize interface: %s", interface);
    ROS_BREAK();
  }

  oob_com_ = new EthercatOobCom(ni_);

  // Initialize Application Layer (AL)
  EtherCAT_DataLinkLayer::instance()->attach(ni_);
  if ((al_ = EtherCAT_AL::instance()) == NULL)
  {
    ROS_FATAL("Unable to initialize Application Layer (AL): %p", al_);
    ROS_BREAK();
  }

  num_slaves_ = al_->get_num_slaves();
  if (num_slaves_ == 0)
  {
    ROS_FATAL("Unable to locate any slaves");
    ROS_BREAK();
  }

  // Initialize Master
  if ((em_ = EtherCAT_Master::instance()) == NULL)
  {
    ROS_FATAL("Unable to initialize EtherCAT_Master: %p", em_);
    ROS_BREAK();
  }

  slaves_ = new EthercatDevice*[num_slaves_];

  // Configure slaves
  unsigned int num_actuators = 0;
  for (unsigned int slave = 0; slave < num_slaves_; ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = em_->get_slave_handler(fsa);
    if (sh == NULL)
    {
      ROS_FATAL("Unable to get slave handler #%d", slave);
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
      num_actuators += slaves_[slave]->has_actuator_;
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
      ROS_BREAK();
    }
  }

  // Allocate buffers to send and receive commands
  buffers_ = new unsigned char[2 * buffer_size_];
  current_buffer_ = buffers_;
  last_buffer_ = buffers_ + buffer_size_;

  // Make sure motors are disabled
  memset(current_buffer_, 0, 2 * buffer_size_);
  em_->txandrx_PD(buffer_size_, current_buffer_);

  // Create HardwareInterface
  hw_ = new HardwareInterface(num_actuators);
  hw_->current_time_ = ros::Time::now();
  last_published_ = hw_->current_time_;

  // Initialize slaves
  set<string> actuator_names;
  for (unsigned int slave = 0, a = 0; slave < num_slaves_; ++slave)
  {
    if (slaves_[slave]->initialize(slaves_[slave]->has_actuator_ ? hw_->actuators_[a] : NULL, allow_unprogrammed) < 0)
    {
      EtherCAT_SlaveHandler *sh = slaves_[slave]->sh_;
      ROS_FATAL("Unable to initialize slave #%d, , product code: %d, revision: %d, serial: %d",
                slave, sh->get_product_code(), sh->get_revision(), sh->get_serial());
      ROS_BREAK();
    }

    // Check for duplicate actuator names
    if (slaves_[slave]->has_actuator_)
    {
      if (!actuator_names.insert(hw_->actuators_[a]->name_).second)
      {
        if (allow_unprogrammed)
        {
          ROS_WARN("Device #%02d: actuator name already in use: '%s'", a, hw_->actuators_[a]->name_.c_str());
        }
        else
        {
          ROS_FATAL("Device #%02d: actuator name already in use: '%s'", a, hw_->actuators_[a]->name_.c_str());
          ROS_BREAK();
        }
      }
    }
    a += slaves_[slave]->has_actuator_;
  }

  // Initialize diagnostic data structures
  publisher_.msg_.status.reserve(num_slaves_ + 1);
  statuses_.reserve(num_slaves_ + 1);
  values_.reserve(10);
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
      rx_late_pkt_rtt_us_avg = c->rx_late_pkt_rtt_us_sum/c->rx_late_pkt;
    }
    status.addf("RX Late Packet Avg RTT", "%f", rx_late_pkt_rtt_us_avg);
  }

  statuses_.push_back(status);

  unsigned char *current = current_buffer_;
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
    publisher_.unlockAndPublish();
  }
}

void EthercatHardware::update(bool reset, bool halt)
{
  unsigned char *current, *last;

  // Convert HW Interface commands to MCB-specific buffers
  current = current_buffer_;

  if (halt)
    halt_motors_ = true;

  if (reset)
  {
    reset_state_ = 2 * num_slaves_;
    halt_motors_ = false;
    diagnostics_.max_roundtrip_ = 0;
  }

  for (unsigned int s = 0, a = 0; s < num_slaves_; ++s)
  {
    if (slaves_[s]->has_actuator_)
    {
      Actuator *act = hw_->actuators_[a];
      slaves_[s]->computeCurrent(act->command_);
      act->state_.last_requested_effort_ = act->command_.effort_;
      act->state_.last_requested_current_ = act->command_.current_;
      slaves_[s]->truncateCurrent(act->command_);
      // Bringup motor boards, one per tick
      if (halt_motors_ || (reset_state_ && (s < reset_state_)))
      {
        bool tmp = act->command_.enable_;
        act->command_.enable_ = (s == (reset_state_/2)) ? true : false;
        act->command_.current_ = 0;
        slaves_[s]->convertCommand(act->command_, current);
        act->command_.enable_ = tmp;
      } else {
        slaves_[s]->convertCommand(act->command_, current);
      }
      current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      ++a;
    }
  }

  // Transmit process data
  double start = ros::Time::now().toSec();
  if (!em_->txandrx_PD(buffer_size_, current_buffer_)) {
    ++diagnostics_.txandrx_errors_;
  }
  diagnostics_.acc_(ros::Time::now().toSec() - start);

  // Transmit new OOB data
  oob_com_->tx();

  // Convert status back to HW Interface
  current = current_buffer_;
  last = last_buffer_;
  for (unsigned int s = 0, a = 0; s < num_slaves_; ++s)
  {
    if (slaves_[s]->has_actuator_)
    {
      Actuator *act = hw_->actuators_[a];
      slaves_[s]->convertState(act->state_, current, last);
      // Don't halt motors during a reset
      if (!slaves_[s]->verifyState(act->state_, current, last) && !reset_state_)
        halt_motors_ = true;
      current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      last += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      ++a;
    }
  }

  if (reset_state_)
    --reset_state_;

  // Update current time
  hw_->current_time_ = ros::Time::now();

  unsigned char *tmp = current_buffer_;
  current_buffer_ = last_buffer_;
  last_buffer_ = tmp;

  if ((hw_->current_time_ - last_published_) > ros::Duration(1.0))
  {
    last_published_ = hw_->current_time_;
    publishDiagnostics();
  }
}

EthercatDevice *
EthercatHardware::configSlave(EtherCAT_SlaveHandler *sh)
{
  static int startAddress = 0x00010000;

  EthercatDevice *p = NULL;
  try
  {
    p = DeviceFactory::Instance().CreateObject(sh->get_product_code(), sh, startAddress);
  }
  catch (Loki::DefaultFactoryError<unsigned int, EthercatDevice>::Exception)
  {}
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


