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

#include <map>
#include <stdio.h>
#include <getopt.h>
#include <sys/mman.h>

#include <tinyxml.h>

#include <ethercat/ethercat_xenomai_drv.h>
#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <al/ethercat_master.h>
#include <al/ethercat_slave_handler.h>

#include "ethercat_hardware/motor_heating_model.h"
#include <ethercat_hardware/wg0x.h>
#include <ethercat_hardware/wg05.h>
#include <ethercat_hardware/wg06.h>
#include <ethercat_hardware/wg021.h>
#include <ethercat_hardware/wg014.h>

#include <boost/crc.hpp>
#include <boost/foreach.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <netinet/in.h>

#include <log4cxx/logger.h>

using namespace ethercat_hardware;

vector<EthercatDevice *> devices;

struct Actuator {
  string motor;
  string board;
  bool enforce_heating_model;
};
typedef pair<string, Actuator> ActuatorPair;
map<string, Actuator> actuators;

struct Config 
{
  Config(const WG0XActuatorInfo &actuator_info, const MotorHeatingModelParametersEepromConfig &heating_config) :
    actuator_info_(actuator_info), heating_config_(heating_config)
  {  }
  Config() {}
  WG0XActuatorInfo actuator_info_;
  MotorHeatingModelParametersEepromConfig heating_config_;
};
typedef pair<string, Config> MotorPair;
map<string, Config> motors;

void init(char *interface)
{
  // open temporary socket to use with ioctl
  int sock = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    int error = errno;
    fprintf(stderr,"Couldn't open temp socket : %s", strerror(error));
    exit(-1);
  }
  
  struct ifreq ifr;
  strncpy(ifr.ifr_name, interface, IFNAMSIZ);
  if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
    int error = errno;
    fprintf(stderr,"Cannot get interface flags for %s: %s\n", interface, strerror(error));
    exit(-1);
  }

  close(sock);
  sock = -1;

  if (!(ifr.ifr_flags & IFF_UP)) {
    fprintf(stderr,"Interface %s is not UP. Try : ifup %s\n", interface, interface);
    exit(-1);
  }
  if (!(ifr.ifr_flags & IFF_RUNNING)) {
    fprintf(stderr,"Interface %s is not RUNNING. Is cable plugged in and device powered?\n", interface);
    exit(-1);
  }

  struct netif *ni;

  // Initialize network interface
  if ((ni = init_ec(interface)) == NULL)
  {
    fprintf(stderr, "Unable to initialize interface: %s\n", interface);
    exit(-1);
  }

  // Initialize Application Layer (AL)
  EtherCAT_DataLinkLayer::instance()->attach(ni);
  EtherCAT_AL *al;
  if ((al = EtherCAT_AL::instance()) == NULL)
  {
    fprintf(stderr, "Unable to initialize Application Layer (AL): %p\n", al);
    exit(-1);
  }

  uint32_t num_slaves = al->get_num_slaves();
  if (num_slaves == 0)
  {
    fprintf(stderr, "Unable to locate any slaves\n");
    exit(-1);
  }

  // Initialize Master
  EtherCAT_Master *em;
  if ((em = EtherCAT_Master::instance()) == NULL)
  {
    fprintf(stderr, "Unable to initialize EtherCAT_Master: %p", em);
    exit(-1);
  }

  static int start_address = 0x00010000;

  for (unsigned int slave = 0; slave < num_slaves; ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = em->get_slave_handler(fsa);
    if (sh == NULL)
    {
      fprintf(stderr, "Unable to get slave handler #%d", slave);
      exit(-1);
    }

    if (sh->get_product_code() == WG05::PRODUCT_CODE)
    {
      WG05 *dev = new WG05();
      dev->construct(sh, start_address);
      devices.push_back(dev);
    }
    else if (sh->get_product_code() == WG06::PRODUCT_CODE)
    {
      WG06 *dev = new WG06();
      dev->construct(sh, start_address);
      devices.push_back(dev);
    }
    else if (sh->get_product_code() == WG021::PRODUCT_CODE)
    {
      WG021 *dev = new WG021();
      dev->construct(sh, start_address);
      devices.push_back(dev);
    }
    else if (sh->get_product_code() == WG014::PRODUCT_CODE)
    {
      WG014 *dev = new WG014();
      dev->construct(sh, start_address);
      devices.push_back(dev);
    }
    else
    {
      devices.push_back(NULL);
    }
  }

  BOOST_FOREACH(EthercatDevice *device, devices)
  {
    if (!device) continue;
    if (!device->sh_->to_state(EC_OP_STATE))
    {
      fprintf(stderr, "Unable set device %d into OP_STATE", device->sh_->get_ring_position());
    }
  }
  
  BOOST_FOREACH(EthercatDevice *device, devices)
  {
    if (!device) continue;
    device->use_ros_ = false;
    device->initialize(NULL, true);
  }
}

string boardName(EthercatDevice *d)
{
  if (dynamic_cast<WG021 *>(d)) {
    return "wg021";
  } else if (dynamic_cast<WG06 *>(d)) {
    return "wg006";
  } else if (dynamic_cast<WG05 *>(d)) {
    return "wg005";
  }
  return "unknown";
}


WG0X* getWGDevice(int device)
{
  uint32_t num_slaves = EtherCAT_AL::instance()->get_num_slaves();
  if ((device >= (int)num_slaves) || (device < 0)) 
  {
    ROS_FATAL("Invalid device number %d.  Must be value between 0 and %d", device, num_slaves-1);
    return NULL;
  }
 
  if (devices[device] == NULL)
  {
    ROS_FATAL("There is no device at position #%d", device);
    return NULL;
  }
  
  WG0X *wg = dynamic_cast<WG0X *>(devices[device]);
  if (wg==NULL) 
  {
    ROS_FATAL("The device a position #%d is not programmable", device);
    return NULL;
  }    

  return wg;
}


bool programDevice(int device, 
                   const Config &config, 
                   char *name, string expected_board, bool enforce_heating_model)
{
  WG0X *wg = getWGDevice(device);
  if (wg == NULL)
  {
    return false;
  }

  string board = boardName(devices[device]);
  if (expected_board != board)
  {
    ROS_FATAL("Device #%02d is a %s, but %s expects a %s\n", device, board.c_str(), name, expected_board.c_str());
    return false;
  }
  ROS_INFO("Programming device %d, to be named: %s\n", device, name);

  WG0XActuatorInfo actuator_info = config.actuator_info_;
  if (strlen(name) >= sizeof(actuator_info.name_))
  {
    ROS_FATAL("Device name '%s' is too long", name);
  }
  strncpy(actuator_info.name_, name, sizeof(actuator_info.name_));
  actuator_info.generateCRC();

  ROS_INFO("Programming actuator version %d.%d", 
           int(actuator_info.major_), int(actuator_info.minor_));

  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
  if (!wg->program(&com, actuator_info))
  {
    ROS_FATAL("Error writing actuator info to device #%d", device);
    return false;
  }

  MotorHeatingModelParametersEepromConfig heating_config = config.heating_config_;
  heating_config.enforce_ = enforce_heating_model;
  heating_config.generateCRC();
  if (!wg->program(&com, config.heating_config_))
  {
    ROS_FATAL("Writing heating model config to device #%d", device);
    return false;
  }

  return true;
}


// Uses ActuatorInfo name already stored in device
// to find appropriate motor heating model config
// Then only updates motor heating model config
bool updateHeatingConfig(int device)
{
  WG0XActuatorInfo actuator_info;
  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
  WG0X *wg = getWGDevice(device);
  if (wg == NULL)
  {
    ROS_WARN("Skipping update of device %d", device);      
    return false;
  }
    
  if (!wg->readActuatorInfoFromEeprom(&com, actuator_info))
  {
    ROS_ERROR("Could not read actuator info from device %d", device);      
    return false;
  }

  if (!actuator_info.verifyCRC())
  {
    ROS_ERROR("Device %d has not actuator configuration", device);
    return false;
  }

  if (actuators.find(actuator_info.name_) == actuators.end())
  {
    ROS_ERROR("Could not find actuator info for device %d with name '%s'",
              device, actuator_info.name_);
    return false;
  }  

  const Actuator &actuator(actuators[actuator_info.name_]);
  const string &motor_name(actuator.motor);
  bool enforce_heating_model = actuator.enforce_heating_model;

  if (motors.find(motor_name) == motors.end())
  {
    ROS_ERROR("Could not find motor '%s' for device %d with actuator name '%s'",
              motor_name.c_str(), device, actuator_info.name_);
    return false;
  }

  const Config &config(motors[motor_name]);  

  if (strcmp(config.actuator_info_.motor_model_, actuator_info.motor_model_) != 0)
  {
    ROS_ERROR("For device %d '%s' : the motor name stored in EEPROM '%s' does not match the motor name '%s' from XML file", 
              device, actuator_info.name_, actuator_info.motor_model_, config.actuator_info_.motor_model_);
    return false;
  }
   
  MotorHeatingModelParametersEepromConfig heating_config = config.heating_config_;
  heating_config.enforce_ = enforce_heating_model;
  heating_config.generateCRC(); 
  if (!wg->program(&com, heating_config))
  {
    ROS_FATAL("Writing heating model config to device #%d", device);
    return false;
  }

  ROS_INFO("Updated device %d (%s) with heating config for motor '%s'", 
           device, actuator_info.name_, config.actuator_info_.motor_model_);

  return true;
}


// Updates heating configuration of all devices
bool updateAllHeatingConfig()
{
  for (unsigned device=0; device<devices.size(); ++device)
  {
    updateHeatingConfig(device);
  }
  return true;
}


static struct
{
  char *program_name_;
  char *interface_;
  char *name_;
  bool program_;
  bool help_;
  int device_;
  string motor_;
  string actuators_;
  string board_;
  bool update_motor_heating_config_;
  bool enforce_heating_model_;
} g_options;

void Usage(string msg = "")
{
  fprintf(stderr, "Usage: %s [options]\n", g_options.program_name_);
  fprintf(stderr, " -i, --interface <i>    Use the network interface <i>\n");
  fprintf(stderr, " -a, --actuators <file> Get the actuator definitions from file (default: actuators.conf)\n");
  fprintf(stderr, " -d, --device <d>       Select the device to program\n");
  fprintf(stderr, " -b, --board <b>        Set the expected board type (wg005, wg006, wg021)\n");
  fprintf(stderr, " -p, --program          Program a motor control board\n");
  fprintf(stderr, " -n, --name <n>         Set the name of the motor control board to <n>\n");
  fprintf(stderr, " -U, --update_heating_config Update motor heating model configuration of all boards\n");
  fprintf(stderr, "     Known actuator names:\n");
  BOOST_FOREACH(ActuatorPair p, actuators)
  {
    string name = p.first;
    fprintf(stderr, "        %s\n", name.c_str());
  }
  fprintf(stderr, " -m, --motor <m>        Set the configuration for motor <m>\n");
  fprintf(stderr, "     Legal motor values are:\n");
  BOOST_FOREACH(MotorPair p, motors)
  {
    const string &name(p.first);
    const WG0XActuatorInfo &info(p.second.actuator_info_);
    fprintf(stderr, "        %s - %s %s\n", name.c_str(), info.motor_make_, info.motor_model_);
  }
  fprintf(stderr, " -h, --help    Print this message and exit\n");
  if (msg != "")
  {
    fprintf(stderr, "Error: %s\n", msg.c_str());
    exit(-1);
  }
  else
  {
    exit(0);
  }
}


bool getDoubleAttribute(TiXmlElement *params, const char* motor_name, const char* param_name, double& value)
{
  const char *val_str = params->Attribute(param_name);
  if (val_str == NULL)
  {
    ROS_ERROR("Attribute '%s' for motor '%s' is not defined", param_name, motor_name);
    return false;
  }
  
  char *endptr=NULL;
  value = strtod(val_str, &endptr);
  if ((endptr == val_str) || (endptr < (val_str+strlen(val_str))))
  {
    ROS_ERROR("Couldn't convert '%s' to double for attribute '%s' of motor '%s'", 
              val_str, param_name, motor_name);
    return false;
  }

  return true;
}

bool getIntegerAttribute(TiXmlElement *params, const char* motor_name, const char* param_name, int& value)
{
  const char *val_str = params->Attribute(param_name);
  if (val_str == NULL)
  {
    ROS_ERROR("Attribute '%s' for motor '%s' is not defined", param_name, motor_name);
    return false;
  }
  
  char *endptr=NULL;
  value = strtol(val_str, &endptr, 0);
  if ((endptr == val_str) || (endptr < (val_str+strlen(val_str))))
  {
    ROS_ERROR("Couldn't convert '%s' to integer for attribute '%s' of motor '%s'", 
              val_str, param_name, motor_name);
    return false;
  }

  return true;
}

bool getStringAttribute(TiXmlElement *params, const char* motor_name, const char* param_name, char* strbuf, unsigned buflen)
{
  const char *val = params->Attribute(param_name);
  if (val == NULL)
  {
    ROS_ERROR("No '%s' attribute for motor '%s'", param_name, motor_name);
    return false;
  }
  if (strlen(val) >= buflen)
  {
    ROS_ERROR("'%s' value '%s' for motor '%s' is too long.  Limit value to %d characters.",
              param_name, val, motor_name, buflen-1);
    return false;
  }
  strncpy(strbuf, val, buflen);
  strbuf[buflen-1] = '\0';
  return true;
}


bool parseConfig(TiXmlElement *config)
{
  TiXmlElement *actuatorElt = config->FirstChildElement("actuators");
  TiXmlElement *motorElt = config->FirstChildElement("motors");

  for (TiXmlElement *elt = actuatorElt->FirstChildElement("actuator");
       elt;
       elt = elt->NextSiblingElement("actuator"))
  {
    const char *name = elt->Attribute("name");
    if (name == NULL)
    {
      ROS_ERROR("Acutuator attribute 'name' not specified");
      return false;
    }

    struct Actuator a;

    const char *motor = elt->Attribute("motor");
    if (motor == NULL)
    {
      ROS_ERROR("For actuator '%s', 'motor' attribute not specified", name);
      return false;
    }
    a.motor = motor;

    const char *board = elt->Attribute("board");
    if (board == NULL)
    {
      ROS_ERROR("For actuator '%s', 'board' attribute not specified", name);
      return false;
    }
    a.board = board;

    const char *enforce_heating_model = elt->Attribute("enforce_heating_model");
    if (enforce_heating_model == NULL)
    {
      // ROS_ERROR("For actuator '%s', 'enforce_heating_model' attribute not specified", name);
      //return false;
      a.enforce_heating_model=false;
    }
    else if (strcmp(enforce_heating_model, "true") == 0)
    {
      a.enforce_heating_model=true;
    }
    else if (strcmp(enforce_heating_model, "false") == 0)
    {
      a.enforce_heating_model=false;
    }
    else 
    {
      ROS_ERROR("For actuator '%s' : 'enforce_heating_model' attribute should be 'true' or 'false' not '%s'",
                name, enforce_heating_model);
      return false;
    }

    actuators[name] = a;
  }

  WG0XActuatorInfo info;
  memset(&info, 0, sizeof(info));
  info.major_ = 0;
  info.minor_ = 2;
  strcpy(info.robot_name_, "PR2");
  ethercat_hardware::MotorHeatingModelParametersEepromConfig heating_config;
  memset(&heating_config, 0, sizeof(heating_config));
  heating_config.major_ = 0;
  heating_config.minor_ = 1;
  heating_config.enforce_ = 0; // default, don't enforce heating model

  for (TiXmlElement *elt = motorElt->FirstChildElement("motor");
       elt;
       elt = elt->NextSiblingElement("motor"))
  {
    bool success = true;
    const char *name = elt->Attribute("name");
    if (name == NULL)
    {
      ROS_ERROR("Motor 'name' attribute is not specified");
      return false;
    }

    TiXmlElement *params = elt->FirstChildElement("params");
    if (params == NULL)
    {
      ROS_ERROR("No 'params' tag available for motor '%s'", name);
      return false;
    }

    TiXmlElement *encoder = elt->FirstChildElement("encoder");
    if (encoder == NULL)
    {
      ROS_ERROR("No 'encoder' tag available for motor '%s'", name);
      return false;
    }

    success &= getStringAttribute(params, name, "make", info.motor_make_, sizeof(info.motor_make_));
    success &= getStringAttribute(params, name, "model", info.motor_model_, sizeof(info.motor_model_));

    double fvalue;

    success &= getDoubleAttribute(params, name, "max_current", fvalue);
    info.max_current_ = fvalue;
    success &= getDoubleAttribute(params, name, "speed_constant", fvalue);
    info.speed_constant_ = fvalue;
    success &= getDoubleAttribute(params, name, "resistance", fvalue);
    info.resistance_ = fvalue;
    success &= getDoubleAttribute(params, name, "motor_torque_constant", fvalue);
    info.motor_torque_constant_ = fvalue;
    success &= getDoubleAttribute(encoder, name, "reduction", fvalue);
    info.encoder_reduction_ = fvalue;

    int ivalue=-1;
    success &= getIntegerAttribute(encoder, name, "pulses_per_revolution", ivalue);
    info.pulses_per_revolution_ = ivalue;
    
    ethercat_hardware::MotorHeatingModelParameters &hmp(heating_config.params_);
    success &= getDoubleAttribute(params, name, "housing_to_ambient_thermal_resistance", fvalue);
    hmp.housing_to_ambient_thermal_resistance_ = fvalue;
    success &= getDoubleAttribute(params, name, "winding_to_housing_thermal_resistance", fvalue);
    hmp.winding_to_housing_thermal_resistance_ = fvalue;
    success &= getDoubleAttribute(params, name, "winding_thermal_time_constant", fvalue);
    hmp.winding_thermal_time_constant_ = fvalue;
    success &= getDoubleAttribute(params, name, "housing_thermal_time_constant", fvalue);
    hmp.housing_thermal_time_constant_ = fvalue;      
    success &= getDoubleAttribute(params, name, "max_winding_temperature", fvalue);
    hmp.max_winding_temperature_       = fvalue;    

    if (!success)
    {
      return false;
    }

    if (motors.find(name) != motors.end())
    {
      ROS_ERROR("Motor named '%s' exists motor than once",name);
      return false;
    }

    motors[name] = Config(info, heating_config);
  }
  return true;
}




int main(int argc, char *argv[])
{

  // Set log level to DEBUG to allow device information to be displayed to
  // output by default
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ros::console::notifyLoggerLevelsChanged();
  
  // Parse options
  g_options.program_name_ = argv[0];
  g_options.device_ = -1;
  g_options.help_ = false;
  g_options.board_ = "";
  g_options.update_motor_heating_config_ = false;
  g_options.enforce_heating_model_ = false;
  while (1)
  {
    static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"interface", required_argument, 0, 'i'},
      {"name", required_argument, 0, 'n'},
      {"device", required_argument, 0, 'd'},
      {"board", required_argument, 0, 'b'},
      {"motor", required_argument, 0, 'm'},
      {"program", no_argument, 0, 'p'},
      {"actuators", required_argument, 0, 'a'},
      {"update_heating_config", no_argument, 0, 'U'},
      {"enforce_heating_model", no_argument, 0, 'H'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "d:b:hi:m:n:pa:UH", long_options, &option_index);
    if (c == -1) break;
    switch (c)
    {
      case 'h':
        g_options.help_ = true;
        break;
      case 'd':
        g_options.device_ = atoi(optarg);
        break;
      case 'b':
        g_options.board_ = optarg;
        break;
      case 'i':
        g_options.interface_ = optarg;
        break;
      case 'n':
        g_options.name_ = optarg;
        break;
      case 'm':
        g_options.motor_ = optarg;
        break;
      case 'p':
        g_options.program_ = 1;
        break;
      case 'a':
        g_options.actuators_ = optarg;
        break;
      case 'U':
        g_options.update_motor_heating_config_ = true;
        break;
      case 'H':
        g_options.enforce_heating_model_ = true;
        break;
    }
  }

  // Parse configuration file
  string filename = "actuators.conf";
  if (g_options.actuators_ != "")
    filename = g_options.actuators_;
  TiXmlDocument xml(filename);

  if (!xml.LoadFile())
  {
    Usage("Unable to load configuration file");
  }

  
  if (!parseConfig(xml.RootElement()))
  {
    exit(EXIT_FAILURE);
  }

  if (g_options.help_)
    Usage();

  if (optind < argc)
  {
    Usage("Extra arguments");
  }

  if (!g_options.interface_)
    Usage("You must specify a network interface");

  // Try to get a raw socket.
  int test_sock = socket(PF_PACKET, SOCK_RAW, htons(0x88A4));
  if ((test_sock < 0) && (errno == EPERM))
  {
    ROS_FATAL("Insufficient priviledges to obtain raw socket.  Try running as root.");
    exit(-1);
  }
  close(test_sock);

  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
  {
    ROS_WARN("mlockall failed : %s", strerror(errno));
  }

  init(g_options.interface_);

  if (g_options.update_motor_heating_config_)
  {
    updateAllHeatingConfig();
  }

  if (g_options.program_)
  {
    string board = "wg005";
    bool enforce_heating_model = false;
    if (!g_options.name_)
      Usage("You must specify a name");
    if (g_options.motor_ == "")
    {
      if (actuators.find(g_options.name_) == actuators.end())
        Usage("No default motor for this name");
      g_options.motor_ = actuators[g_options.name_].motor;
      board = actuators[g_options.name_].board;
      enforce_heating_model = actuators[g_options.name_].enforce_heating_model;
    }
    if (g_options.board_ != "") {
      board = g_options.board_;
    }
    if (g_options.device_ == -1)
      Usage("You must specify a device #");
    if (motors.find(g_options.motor_) == motors.end())
      Usage("You must specify a valid motor");
    if (g_options.enforce_heating_model_)
      enforce_heating_model = true;

    programDevice(g_options.device_, motors[g_options.motor_], g_options.name_, board, enforce_heating_model);
  }

  return 0;
}


