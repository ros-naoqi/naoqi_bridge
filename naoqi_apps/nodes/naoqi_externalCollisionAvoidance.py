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
from naoqi_bridge_msgs.msg import (
    MoveFailed, 
    ChainVelocityClipped)
from naoqi_bridge_msgs.srv import ( 
    GetChainClosestObstaclePositionResponse,
    GetChainClosestObstaclePosition,
    SetTangentialSecurityDistanceResponse,
    SetTangentialSecurityDistance, 
    GetTangentialSecurityDistanceResponse,
    GetTangentialSecurityDistance, 
    SetOrthogonalSecurityDistanceResponse,
    SetOrthogonalSecurityDistance,
    GetOrthogonalSecurityDistanceResponse,
    GetOrthogonalSecurityDistance,
    SetExternalCollisionProtectionEnabledResponse,
    SetExternalCollisionProtectionEnabled,
    GetExternalCollisionProtectionEnabledResponse,
    GetExternalCollisionProtectionEnabled,)

class NaoqiExternalCollisionAvoidance(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_externalCollisionAvoidance')
        self.connectNaoQi()
        
        self.moveFailed = MoveFailed()
        self.previousMoveFailed = ""
        self.chainVelocityClipped = ChainVelocityClipped()
        self.previousChainVelocityClipped = ""
        self.getChainClosesObstaclePositionSrv = rospy.Service("get_chain_closest_obstacle_position",GetChainClosestObstaclePosition , self.handleGetChainClosestObstaclePosition) 
        self.getExternalCollisionProtectionEnabledSrv = rospy.Service("get_external_collision_protection_status", GetExternalCollisionProtectionEnabled, self.handleGetExternalCollisionProtectionEnabled)
        self.getOrthogonalSecurityDistanceSrv = rospy.Service("get_orthogonal_distance", GetOrthogonalSecurityDistance, self.handleGetOrthogonalSecurityDistance)
        self.getTangentialSecurityDistanceSrv = rospy.Service("get_tangential_distance", GetTangentialSecurityDistance, self.handleGetTangentialSecurityDistance)
        self.setExternalCollisionProtectionEnabledSrv = rospy.Service("set_external_collision_protection_status", SetExternalCollisionProtectionEnabled, self.handleSetExternalCollisionProtectionEnabled)
        self.setOrthogonalSecurityDistanceSrv = rospy.Service("set_orthogonal_distance", SetOrthogonalSecurityDistance, self.handleSetOrthogonalSecurityDistance)
        self.setTangentialSecurityDistanceSrv = rospy.Service("set_tangential_distance", SetTangentialSecurityDistance, self.handleSetTangentialSecurityDistance)
        self.chainVelocityClippedPub = rospy.Publisher("chain_velocity_clipped", ChainVelocityClipped, queue_size=10) 
        self.moveFailedPub = rospy.Publisher("move_failed", MoveFailed, queue_size=10)       
        rospy.loginfo("naoqi_externalCollisionAvoidance initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.motionProxy = self.get_proxy("ALMotion")
        if self.memProxy is None or self.motionProxy is None:
            exit(1)
        
    def handleGetChainClosestObstaclePosition(self, req):
        try:
            if req.arm.data == 0:
                arm="LArm"
            else:
                arm="RArm"
            position = self.motionProxy.getChainClosestObstaclePosition(arm, req.frame.data)
            res = GetChainClosestObstaclePositionResponse()
            res.obstacle_position.x = position[0]
            res.obstacle_position.y = position[1]
            res.obstacle_position.z = position[2]
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
    def handleSetOrthogonalSecurityDistance(self, req):
        try:
            res = SetOrthogonalSecurityDistanceResponse()
            if (req.orthogonal_distance > 0.001 or req.orthogonal_distance == 0.001):
                self.motionProxy.setOrthogonalSecurityDistance(req.orthogonal_distance)
                rospy.loginfo("Orthogonal security distance is set to " + str(req.orthogonal_distance))
                res.success = True
                return res
            else:
                rospy.loginfo("Orthogonal security distance should be 0.001, or more than 0.001")
                res.success = False
                return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleSetTangentialSecurityDistance(self, req):
        try:
            res = SetTangentialSecurityDistanceResponse()
            if (req.tangential_distance > 0.001 or req.tangential_distance == 0.001):
                self.motionProxy.setTangentialSecurityDistance(req.tangential_distance)
                rospy.loginfo("Tangential security distance is set to " + str(req.tangential_distance))
                res.success = True
                return res
            else:
                rospy.loginfo("Tangential distance should be 0.001, or more than 0.001")
                res.success = False
                return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleGetOrthogonalSecurityDistance(self, req):
        try:
            rospy.loginfo("Current Orthogonal Security Distance: " + str(self.motionProxy.getOrthogonalSecurityDistance()))
            res = GetOrthogonalSecurityDistanceResponse()
            res.orthogonal_distance = self.motionProxy.getOrthogonalSecurityDistance()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTangentialSecurityDistance(self, req):
        try:
            rospy.loginfo("Current Tangential Security Distance: " + str(self.motionProxy.getTangentialSecurityDistance()))
            res = GetTangentialSecurityDistanceResponse()
            res.tangential_distance = self.motionProxy.getTangentialSecurityDistance()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetExternalCollisionProtectionEnabled(self, req):
        try:
            if req.name.data == 0:
                target_name = "All"
            if req.name.data == 1:
                target_name = "Move"
            if req.name.data == 2:
                target_name = "Arms"
            if req.name.data == 3:
                target_name = "LArm"
            if req.name.data == 4:
                target_name = "RArm"
            res = SetExternalCollisionProtectionEnabledResponse()
            res.success = False
            if self.motionProxy.setExternalCollisionProtectionEnabled(target_name, req.status) == None:
                res.success = True
                if req.status == True:
                    rospy.loginfo("Current External Collision Protection Status of " + target_name + ": Enabled")
                else:
                    rospy.loginfo("Current External Collision Protection Status of " + target_name + ": Disabled")
                    if (target_name == "All" or target_name == "Move") and req.status == False:
                        rospy.loginfo("CAUTION: YOU DEACTIVATED SAFETY REFLEXES. BE CAREFUL TO THE SAFETY OF THE ROBOT AND PEOPLE.")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleGetExternalCollisionProtectionEnabled(self, req):
        try:
            if req.name.data == 0:
                target_name = "All"
            if req.name.data == 1:
                target_name = "Move"
            if req.name.data == 2:
                target_name = "Arms"
            if req.name.data == 3:
                target_name = "LArm"
            if req.name.data == 4:
                target_name = "RArm"
            res = GetExternalCollisionProtectionEnabledResponse()
            res.status = self.motionProxy.getExternalCollisionProtectionEnabled(target_name)
            rospy.loginfo("Current External Collision Protection Status of " + target_name + ": " + str (res.status))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def run(self):
        while self.is_looping():
            try:
                moveFailedData = self.memProxy.getData("ALMotion/MoveFailed")
                if moveFailedData != self.previousMoveFailed:
                    self.moveFailed.header.stamp = rospy.get_rostime()
                    self.moveFailed.header.frame_id = "FRAME_WORLD"
                    self.moveFailed.cause = moveFailedData[0]
                    self.moveFailed.status = moveFailedData[1]
                    self.moveFailed.obstacle_position.data = []
                    if moveFailedData[2]:
                        for i in range (len(moveFailedData[2])):
                            self.moveFailed.obstacle_position.data.append((moveFailedData[2])[i])
                    self.moveFailedPub.publish(self.moveFailed)
                    self.previousMoveFailed = moveFailedData
                chainVelocityClippedData = self.memProxy.getData("ALMotion/Safety/ChainVelocityClipped")
                if chainVelocityClippedData != None and chainVelocityClippedData != self.previousChainVelocityClipped:
                    for i in range(len(chainVelocityClippedData)):
                        self.chainVelocityClipped.header.stamp = rospy.get_rostime()
                        self.chainVelocityClipped.header.frame_id = "FRAME_WORLD"
                        if (chainVelocityClippedData[i])[0] == "Head":
                            self.chainVelocityClipped.chain.data = 0
                        if (chainVelocityClippedData[i])[0] == "Leg":
                            self.chainVelocityClipped.chain.data = 1
                        if (chainVelocityClippedData[i])[0] == "LArm":
                            self.chainVelocityClipped.chain.data = 2
                        if (chainVelocityClippedData[i])[0] == "RArm":
                            self.chainVelocityClipped.chain.data = 3
                        self.chainVelocityClipped.obstacle_position.x = ((chainVelocityClippedData[i])[1])[0]
                        self.chainVelocityClipped.obstacle_position.y = ((chainVelocityClippedData[i])[1])[1]
                        self.chainVelocityClipped.obstacle_position.z = ((chainVelocityClippedData[i])[1])[2]
                        self.chainVelocityClippedPub.publish(self.chainVelocityClipped) 
                    self.previousChainVelocityClipped = chainVelocityClippedData
            except RuntimeError, e:
                print "Error accessing ALMemory and ALMotion, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    externalCollisionAvoidance = NaoqiExternalCollisionAvoidance()
    externalCollisionAvoidance.start()
    rospy.spin()
