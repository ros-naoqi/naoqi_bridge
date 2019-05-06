#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    EmptyResponse,
    Empty)

class NaoqiSystem(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_system')
        self.connectNaoQi()
        self.rebootSrv = rospy.Service("reboot", Empty, self.handleRebootSrv)
        rospy.loginfo("naoqi_system initialized")
    
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.systemProxy = self.get_proxy("ALSystem")
        if self.systemProxy is None:
            rospy.logerr("Could not get a proxy to ALSystem")
            exit(1)

    def handleRebootSrv(self, req):
        try:
            self.systemProxy.reboot()
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

if __name__ == '__main__':
    system = NaoqiSystem()
    rospy.spin()
