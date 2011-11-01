#include <boost/program_options.hpp>
#include <alerror/alerror.h>

#include <ros/ros.h> // for logging macros

#include <iostream>

#include "nao_node.h"

using namespace std;

NaoNode::NaoNode(int argc, char ** argv) : 
    m_pip("nao.local"),
    m_ip("0.0.0.0"),
    m_port(16712),
    m_pport(9559),
    m_brokerName("NaoROSBroker")
{
    parse_command_line(argc, argv);
}

NaoNode::~NaoNode()
{}

void NaoNode::parse_command_line(int argc, char ** argv)
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



bool NaoNode::connectNaoQi()
{
   // Need this to for SOAP serialization of floats to work
   setlocale(LC_NUMERIC, "C");
   // A broker needs a name, an IP and a port:
   // FIXME: would be a good idea to look for a free port first
   // listen port of the broker (here an anything)
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

