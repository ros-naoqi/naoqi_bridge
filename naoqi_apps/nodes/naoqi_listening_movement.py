#!/usr/bin/env python

#                                                                             
#  Copyright 2016 Aldebaran                                                   
#                                                                             
#  Licensed under the Apache License, Version 2.0 (the "License");            
#  you may not use this file except in compliance with the License.           
#  You may obtain a copy of the License at                                    
#                                                                             
#      http://www.apache.org/licenses/LICENSE-2.0                             
#                                                                             
#  Unless required by applicable law or agreed to in writing, software        
#  distributed under the License is distributed on an "AS IS" BASIS,          
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   
#  See the License for the specific language governing permissions and        
#  limitations under the License.                                             
#                                                                             
# 
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    SetBoolResponse,
    SetBool,
    TriggerResponse,
    Trigger
)
from distutils.version import LooseVersion

class NaoqiListeningMovement(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_listening_movement')
        self.connectNaoQi()
        
        self.SetEnabledSrv = rospy.Service("listening_movement/set_enabled", SetBool, self.handleSetEnabledSrv)
        self.IsEnabledSrv = rospy.Service("listening_movement/is_enabled", Trigger, self.handleIsEnabledSrv)
        rospy.loginfo("naoqi_listening_movement initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.systemProxy = self.get_proxy("ALSystem")
        if self.systemProxy is None:
            rospy.logerr("Could not get a proxy to ALSystem")
            exit(1)
        else:
            if LooseVersion(self.systemProxy.systemVersion()) < LooseVersion("2.4"):
                rospy.logerr("Naoqi version of your robot is " + str(self.systemProxy.systemVersion()) + ", which doesn't have a proxy to ALListeningMovement.")
                exit(1)
            else:
                self.listeningMovementProxy = self.get_proxy("ALListeningMovement")
                if self.listeningMovementProxy is None:
                    rospy.logerr("Could not get a proxy to ALListeningMovement")
                    exit(1)
    
    def handleSetEnabledSrv(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.listeningMovementProxy.setEnabled(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleIsEnabledSrv(self, req):
        try:
            res = TriggerResponse()
            res.success = self.listeningMovementProxy.isEnabled()
            return  res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

if __name__ == '__main__':
    listening_movement = NaoqiListeningMovement()
    rospy.spin()
