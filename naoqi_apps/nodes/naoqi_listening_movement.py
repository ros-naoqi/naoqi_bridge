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
from naoqi_bridge_msgs.srv import ( 
    IsActiveResponse,
    IsActive,
    SetEnabledResponse,
    SetEnabled,)

class NaoqiListeningMovement(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_listening_movement')
        self.connectNaoQi()
        
        self.SetEnabledSrv = rospy.Service("listening_movement_set_enabled", SetEnabled, self.handleSetEnabledSrv)
        self.IsEnabledSrv = rospy.Service("listening_movement_is_enabled", IsActive, self.handleIsEnabledSrv)
        self.IsRunningSrv = rospy.Service("listening_movement_is_running", IsActive, self.handleIsRunningSrv)
        rospy.loginfo("naoqi_listening_movement initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.listeningMovementProxy = self.get_proxy("ALListeningMovement")
        if self.listeningMovementProxy is None:
            exit(1)
    
    def handleSetEnabledSrv(self, req):
        res = SetEnabledResponse()
        res.success = False
        try:
            self.listeningMovementProxy.setEnabled(req.enabled)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleIsEnabledSrv(self, req):
        try:
            res = IsActiveResponse()
            res.status = self.listeningMovementProxy.isEnabled()
            return  res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsRunningSrv(self, req):
        try:
            res = IsActiveResponse()
            res.status = self.listeningMovementProxy.isRunning()
            return  res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def run(self):
        while self.is_looping():
            try:
                pass
            except RuntimeError, e:
                print "Error accessing ALListeningMovement, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    listening_movement = NaoqiListeningMovement()
    listening_movement.start()
    rospy.spin()
