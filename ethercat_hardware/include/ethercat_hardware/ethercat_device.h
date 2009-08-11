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

#ifndef ETHERCAT_DEVICE_H
#define ETHERCAT_DEVICE_H

#include <vector>

#include <tinyxml/tinyxml.h>

#include <ethercat/ethercat_defs.h>
#include <al/ethercat_slave_handler.h>

#include <hardware_interface/hardware_interface.h>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <loki/Factory.h>
#include <loki/Sequence.h>

using namespace std;

class EthercatDevice
{
public:
  EthercatDevice(EtherCAT_SlaveHandler *sh, bool has_actuator = false, int command_size = 0, int status_size = 0) :
    sh_(sh), has_actuator_(has_actuator), command_size_(command_size), status_size_(status_size) {}

  virtual ~EthercatDevice() {}

  virtual int initialize(Actuator *, bool allow_unprogrammed=0) = 0;

  virtual void convertCommand(ActuatorCommand &command, unsigned char *buffer) = 0;
  virtual void convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer) = 0;

  virtual void computeCurrent(ActuatorCommand &command) = 0;
  virtual void truncateCurrent(ActuatorCommand &command) = 0;
  virtual bool verifyState(ActuatorState &state, unsigned char *this_buffer, unsigned char *prev_buffer) = 0;

  virtual void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *) = 0;

  EtherCAT_SlaveHandler *sh_;
  bool has_actuator_;
  unsigned int command_size_;
  unsigned int status_size_;
};


typedef Loki::SingletonHolder
<
  Loki::Factory< EthercatDevice, EC_UDINT, Loki::Seq<EtherCAT_SlaveHandler *, int&> >,
  Loki::CreateUsingNew,
  Loki::LongevityLifetime::DieAsSmallObjectChild
> DeviceFactory;

template< class T> T* deviceCreator(EtherCAT_SlaveHandler *sh, int &addr) {return new T(sh, addr);}

#endif /* ETHERCAT_DEVICE_H */
