#!/usr/bin/env python

#                                                                             
#  Copyright 2017 Aldebaran                                                   
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

class NaoqiBasicAwareness(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_basic_awareness')
        self.connectNaoQi()
        
        self.SetEnabledSrv = rospy.Service("basic_awareness_set_enabled", SetBool, self.handleSetEnabledSrv)
        self.IsEnabledSrv = rospy.Service("basic_awareness_is_enabled", Trigger, self.handleIsEnabledSrv)
        rospy.loginfo("naoqi_basic_awareness initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.basicAwarenessProxy = self.get_proxy("ALBasicAwareness")
        if self.basicAwarenessProxy is None:
            exit(1)
    
    def handleSetEnabledSrv(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.basicAwarenessProxy.setEnabled(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleIsEnabledSrv(self, req):
        try:
            res = TriggerResponse()
            res.success = self.basicAwarenessProxy.isEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def run(self):
        while self.is_looping():
            try:
                pass
            except RuntimeError, e:
                print "Error accessing ALBasicAwareness, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    basic_awareness = NaoqiBasicAwareness()
    basic_awareness.start()
    rospy.spin()
