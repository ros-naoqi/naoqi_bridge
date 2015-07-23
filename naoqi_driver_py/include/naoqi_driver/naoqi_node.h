#ifndef NAOQI_NODE_H
#define NAOQI_NODE_H

#include <string>
#include <alcommon/albroker.h>

class NaoqiNode
{
   public:
      NaoqiNode(int argc, char ** argv);
      ~NaoqiNode();
      bool connectNaoQi();
      bool connectLocalNaoQi();
      void parse_command_line(int argc, char ** argv);
   protected:
      std::string m_pip;
      std::string m_ip;
      int m_port;
      int m_pport;
      std::string m_brokerName;
      boost::shared_ptr<AL::ALBroker> m_broker;


};

#endif
