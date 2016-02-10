#!/usr/bin/env python

#                                                                             
#  Copyright 2015 Aldebaran                                                   
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
from std_msgs.msg import String 
from std_srvs.srv import (
    EmptyResponse,
    Empty)
from naoqi_bridge_msgs.srv import ( 
    TangentialSecurityDistanceResponse,
    TangentialSecurityDistance, 
    OrthogonalSecurityDistanceResponse,
    OrthogonalSecurityDistance,
    SetExternalCollisionProtectionEnabledResponse,
    SetExternalCollisionProtectionEnabled,
    GetExternalCollisionProtectionEnabledResponse,
    GetExternalCollisionProtectionEnabled,)

class NaoqiExternalCollisionAvoidance(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_externalCollisionAvoidance')
        self.connectNaoQi()
        
        # init. messages:
        self.moveFailed = String()
        self.previousMoveFailedInfomation = ""
        self.setOrthogonalSecurityDistanceSrv = rospy.Service("set_orthogonal_distance", OrthogonalSecurityDistance, self.handleSetOrthogonalSecurityDistanceSrv)
        self.setTangentialSecurityDistanceSrv = rospy.Service("set_tangential_distance", TangentialSecurityDistance, self.handleSetTangentialSecurityDistanceSrv)
        self.getMoveFailedInformationSrv = rospy.Service("move_failed_information", Empty, self.handleMoveFailedInformationSrv)
        self.setOrthogonalSecurityDistanceSrv = rospy.Service("get_orthogonal_distance", Empty, self.handleGetOrthogonalSecurityDistanceSrv)
        self.setTangentialSecurityDistanceSrv = rospy.Service("get_tangential_distance", Empty, self.handleGetTangentialSecurityDistanceSrv)
        self.setExternalCollisionProtectionEnabledSrv = rospy.Service("set_external_collision_protection_status", SetExternalCollisionProtectionEnabled, self.handleSetExternalCollisionProtectionEnabled)
        self.getExternalCollisionProtectionEnabledSrv = rospy.Service("get_external_collision_protection_status", GetExternalCollisionProtectionEnabled, self.handleGetExternalCollisionProtectionEnabled)
        rospy.loginfo("naoqi_externalCollisionAvoidance initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.motionProxy = self.get_proxy("ALMotion")
        if self.memProxy is None or self.motionProxy is None:
            exit(1)
        
    def handleSetOrthogonalSecurityDistanceSrv(self, req):
        try:
            if (req.orthogonal_distance > 0.001 or req.orthogonal_distance == 0.001):
                self.motionProxy.setOrthogonalSecurityDistance(req.orthogonal_distance.data)
                rospy.loginfo("Orthogonal security distance is set to " + str(req.orthogonal_distance.data))
                return OrthogonalSecurityDistanceResponse()
            else:
                rospy.loginfo("Orthogonal security distance should be 0.001, or more than 0.001")
                return OrthogonalSecurityDistanceResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleSetTangentialSecurityDistanceSrv(self, req):
        try:
            if (req.tangential_distance > 0.001 or req.tangential_distance == 0.001):
                self.motionProxy.setTangentialSecurityDistance(req.tangential_distance.data)
                rospy.loginfo("Tangential security distance is set to " + str(req.tangential_distance.data))
                return TangentialSecurityDistanceResponse()
            else:
                rospy.loginfo("Tangential distance should be 0.001, or more than 0.001")
                return TangentialSecurityDistanceResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleGetOrthogonalSecurityDistanceSrv(self, req):
        try:
            rospy.loginfo("Current Orthogonal Security Distance: " + str(self.motionProxy.getOrthogonalSecurityDistance()))
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTangentialSecurityDistanceSrv(self, req):
        try:
            rospy.loginfo("Current Tangential Security Distance: " + str(self.motionProxy.getTangentialSecurityDistance()))
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleMoveFailedInformationSrv(self, req):
        if self.memProxy.getData("ALMotion/MoveFailed") != self.previousMoveFailedInfomation:
            moveFailedData = self.memProxy.getData("ALMotion/MoveFailed")
            self.previousMoveFailedInformation = moveFailedData
            moveFailedMessage = "I CANNOT MOVE\n Cause of move failed: " + moveFailedData[0]
            if moveFailedData[1] == 0:
                moveFailedMessage  += ", Status: move not started" 
            if moveFailedData[1] == 1:
                moveFailedMessage += ", Status: move started but stopped" 
            moveFailedMessage += ", Obstacle position: "+ str(moveFailedData[2])
            rospy.loginfo(moveFailedMessage)
            return EmptyResponse()

    def handleSetExternalCollisionProtectionEnabled(self, req):
        try:
            self.motionProxy.setExternalCollisionProtectionEnabled(req.name.data, req.status)
            if self.motionProxy.getExternalCollisionProtectionEnabled(req.name.data)== True: 
                rospy.loginfo("Current External Collision Protection Status of " + req.name.data + ": Enabled")
            else:
                rospy.loginfo("Current External Collision Protection Status of " + req.name.data + ": Disabled")
            if (req.name.data == "All" or req.name.data == "Move") and req.status == False:
                rospy.loginfo("CAUTION: YOU DEACTIVATED SAFETY REFLEXES. BE CAREFUL TO THE SAFETY OF THE ROBOT AND PEOPLE.")
            return SetExternalCollisionProtectionEnabledResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetExternalCollisionProtectionEnabled(self, req):
        try:
            if self.motionProxy.getExternalCollisionProtectionEnabled(req.name.data)== True: 
                rospy.loginfo("Current External Collision Protection Status of " + req.name.data + ": Enabled")
            else:
                rospy.loginfo("Current External Collision Protection Status of " + req.name.data + ": Disabled")
            return GetExternalCollisionProtectionEnabledResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def run(self):
        while self.is_looping():
            try:
                pass
            except RuntimeError, e:
                print "Error accessing ALMemory and ALMotion, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    externalCollisionAvoidance = NaoqiExternalCollisionAvoidance()
    externalCollisionAvoidance.start()
    rospy.spin()
