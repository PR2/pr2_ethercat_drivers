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

#include <ethercat/ethercat_xenomai_drv.h>
#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <al/ethercat_master.h>
#include <al/ethercat_slave_handler.h>

#include <ethercat_hardware/wg0x.h>
#include <ethercat_hardware/wg014.h>

#include <boost/crc.hpp>
#include <boost/foreach.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <netinet/in.h>

vector<EthercatDevice *> devices;

struct Actuator {
  string motor;
  string board;
};
typedef pair<string, Actuator> ActuatorPair;
map<string, Actuator> actuators;

typedef pair<string, WG0XActuatorInfo> MotorPair;
map<string, WG0XActuatorInfo> motors;

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

void programDevice(int device, WG0XActuatorInfo &config, char *name, string expected_board)
{
  uint32_t num_slaves = EtherCAT_AL::instance()->get_num_slaves();
  if ((device >= (int)num_slaves) || (device < 0)) {
    ROS_FATAL("Invalid device number %d.  Must be value between 0 and %d", device, num_slaves-1);
    return;
  }
 
  if (devices[device])
  {
    WG0X *wg = dynamic_cast<WG0X *>(devices[device]);

    if (wg) {
      string board = boardName(devices[device]);
      if (expected_board != board) {
        ROS_FATAL("Device #%02d is a %s, but %s expects a %s\n", device, board.c_str(), name, expected_board.c_str());
        return;
      }
      ROS_INFO("Programming device %d, to be named: %s\n", device, name);
      strcpy(config.name_, name);
      boost::crc_32_type crc32;
      crc32.process_bytes(&config, sizeof(config)-sizeof(config.crc32_));
      config.crc32_ = crc32.checksum();
      wg->program(&config);
    }
    else
    {
      ROS_FATAL("The device a position #%d is not programmable", device);
    }
  }
  else
  {
    ROS_FATAL("There is no device at position #%d", device);
  }
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
    string name = p.first;
    WG0XActuatorInfo info = p.second;
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

void parseConfig(TiXmlElement *config)
{
  TiXmlElement *actuatorElt = config->FirstChildElement("actuators");
  TiXmlElement *motorElt = config->FirstChildElement("motors");

  for (TiXmlElement *elt = actuatorElt->FirstChildElement("actuator");
       elt;
       elt = elt->NextSiblingElement("actuator"))
  {
    const char *name = elt->Attribute("name");
    struct Actuator a;
    a.motor = elt->Attribute("motor");
    a.board = elt->Attribute("board");
    actuators[name] = a;
  }

  WG0XActuatorInfo info;
  memset(&info, 0, sizeof(info));
  info.minor_ = 2;
  strcpy(info.robot_name_, "PR2");
  for (TiXmlElement *elt = motorElt->FirstChildElement("motor");
       elt;
       elt = elt->NextSiblingElement("motor"))
  {
    const char *name = elt->Attribute("name");
    TiXmlElement *params = elt->FirstChildElement("params");
    TiXmlElement *encoder = elt->FirstChildElement("encoder");

    strcpy(info.motor_make_, params->Attribute("make"));
    strcpy(info.motor_model_, params->Attribute("model"));

    info.max_current_ = atof(params->Attribute("max_current"));
    info.speed_constant_ = atof(params->Attribute("speed_constant"));
    info.resistance_ = atof(params->Attribute("resistance"));
    info.motor_torque_constant_ = atof(params->Attribute("motor_torque_constant"));

    info.pulses_per_revolution_ = atoi(encoder->Attribute("pulses_per_revolution"));
    info.encoder_reduction_ = atoi(encoder->Attribute("reduction"));

    motors[name] = info;
  }
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
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "d:b:hi:m:n:pa:", long_options, &option_index);
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
    }
  }

  // Parse configuration file
  string filename = "actuators.conf";
  if (g_options.actuators_ != "")
    filename = g_options.actuators_;
  TiXmlDocument xml(filename);

  if (xml.LoadFile())
  {
    parseConfig(xml.RootElement());
  }
  else
  {
    Usage("Unable to load configuration file");
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

  if (g_options.program_)
  {
    string board = "wg005";
    if (!g_options.name_)
      Usage("You must specify a name");
    if (g_options.motor_ == "")
    {
      if (actuators.find(g_options.name_) == actuators.end())
        Usage("No default motor for this name");
      g_options.motor_ = actuators[g_options.name_].motor;
      board = actuators[g_options.name_].board;
    }
    if (g_options.board_ != "") {
      board = g_options.board_;
    }
    if (g_options.device_ == -1)
      Usage("You must specify a device #");
    if (motors.find(g_options.motor_) == motors.end())
      Usage("You must specify a valid motor");

    programDevice(g_options.device_, motors[g_options.motor_], g_options.name_, board);
  }

  return 0;
}
