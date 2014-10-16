#include <boost/program_options.hpp>
#include <alerror/alerror.h>
#include <alcommon/albrokermanager.h>

#include <ros/ros.h> // for logging macros

#include <iostream>

#include <naoqi_driver/naoqi_node.h>

using namespace std;

NaoqiNode::NaoqiNode(int argc, char ** argv) :
    m_pip("nao.local"),
    m_ip("0.0.0.0"),
    m_port(0),
    m_pport(9559),
    m_brokerName("NaoROSBroker")
{
    parse_command_line(argc, argv);
}

NaoqiNode::~NaoqiNode()
{}

void NaoqiNode::parse_command_line(int argc, char ** argv)
{
   string pip;
   string ip;
   int pport;
   int port;
   boost::program_options::options_description desc("Configuration");
   desc.add_options()
      ("help", "show this help message")
      ("ip", boost::program_options::value<string>(&ip)->default_value(m_ip),
       "IP/hostname of the broker")
      ("port", boost::program_options::value<int>(&port)->default_value(m_port),
       "Port of the broker")
      ("pip", boost::program_options::value<string>(&pip)->default_value(m_pip),
       "IP/hostname of parent broker")
      ("pport", boost::program_options::value<int>(&pport)->default_value(m_pport),
       "port of parent broker")
      ;
   boost::program_options::variables_map vm;
   boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
   boost::program_options::notify(vm);
   m_port = vm["port"].as<int>();
   m_pport = vm["pport"].as<int>();
   m_pip = vm["pip"].as<string>();
   m_ip = vm["ip"].as<string>();
   ROS_DEBUG_STREAM("pip is " << m_pip);
   ROS_DEBUG_STREAM("ip is " << m_ip);
   ROS_DEBUG_STREAM("port is " << m_port);
   ROS_DEBUG_STREAM("pport is " << m_pport);

   if (vm.count("help")) {
      cout << desc << "\n";
      return ;
   }
}

bool NaoqiNode::connectLocalNaoQi() {
   try
   {
      // The manager might not exist in this process wich causes this to crash.
      boost::shared_ptr<AL::ALBrokerManager> manager =
        AL::ALBrokerManager::getInstance();
      // This will crash if there is no broker registered.
      m_broker = manager->getRandomBroker();
   }
   catch(...)
   {
      // There is no broker manager running in this process.
      return false;
   }

   // Fill in members
   m_ip = m_broker->getIP();
   m_port = m_broker->getPort();
   m_pip = m_broker->getParentIP();
   m_pport = m_broker->getParentPort();
   m_brokerName = m_broker->getName();

  return true;
}

bool NaoqiNode::connectNaoQi()
{
   // Need this to for SOAP serialization of floats to work
   setlocale(LC_NUMERIC, "C");
   try
   {
      m_broker = AL::ALBroker::createBroker(m_brokerName, m_ip, m_port, m_pip, m_pport, false);
   }
   catch(const AL::ALError& e)
   {
      ROS_ERROR( "Failed to connect broker to: %s:%d",m_pip.c_str(),m_port);
      //AL::ALBrokerManager::getInstance()->killAllBroker();
      //AL::ALBrokerManager::kill();
      return false;
   }
   ROS_INFO("NAOqi broker ready.");
   return true;
}

