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
    EmptyResponse,
    Empty)
from naoqi_bridge_msgs.srv import ( 
    MoveIsActiveResponse,
    MoveIsActive,
    GetRobotPositionResponse,
    GetRobotPosition,
    GetNextRobotPositionResponse,
    GetNextRobotPosition,
    GetRobotVelocityResponse,
    GetRobotVelocity,
    GetArmsEnabledResponse,
    GetArmsEnabled,
    SetArmsEnabledResponse,
    SetArmsEnabled)

class NaoqiLocomotionControl(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_motion')
        self.connectNaoQi()

        self.moveInitSrv = rospy.Service("move_init", Empty, self.handleMoveInit)
        self.waitUnitilMoveIsFinishedSrv = rospy.Service("wait_until_move_is_finished", Empty, self.handleWaitUntilMoveIsFinished)
        self.moveIsActiveSrv = rospy.Service("move_is_active", MoveIsActive, self.handleMoveIsActive)
        self.stopMoveSrv = rospy.Service("stop_move", Empty, self.handleStopMove)
        self.getRobotPositionSrv = rospy.Service("get_robot_position", GetRobotPosition, self.handleGetRobotPosition)
        self.getNextRobotPositionSrv = rospy.Service("get_next_robot_position", GetNextRobotPosition, self.handleGetNextRobotPosition)
        self.getRobotVelocitySrv = rospy.Service("get_robot_velocity", GetRobotVelocity, self.handleGetRobotVelocity)
        self.getArmsEnabledSrv = rospy.Service("get_move_arms_enabled", GetArmsEnabled, self.handleGetMoveArmsEnabled)        
        self.setArmsEnabledSrv = rospy.Service("set_move_arms_enabled", SetArmsEnabled, self.handleSetMoveArmsEnabled)
        rospy.loginfo("naoqi_motion initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.motionProxy = self.get_proxy("ALMotion")
        if self.memProxy is None or self.motionProxy is None:
            exit(1)
            
    def handleMoveInit(self, req):
        try:
            self.motionProxy.moveInit()
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleWaitUntilMoveIsFinished(self, req):
        try:
            self.motionProxy.waitUntilMoveIsFinished()
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleMoveIsActive(self, req):
        try:
            res = MoveIsActiveResponse()
            res.status = self.motionProxy.moveIsActive()
            return  res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
    def handleStopMove(self, req):
        try:
            self.motionProxy.stopMove()
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
    def handleGetRobotPosition(self, req):
        try:
            getRobotPosition = self.motionProxy.getRobotPosition(req.use_sensors_values)
            res = GetRobotPositionResponse()
            res.pose.x = getRobotPosition[0]
            res.pose.y = getRobotPosition[1]
            res.pose.theta = getRobotPosition[2]
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetNextRobotPosition(self, req):
        try:
            getNextRobotPosition = self.motionProxy.getNextRobotPosition()
            res = GetNextRobotPositionResponse()
            res.pose.x = getNextRobotPosition[0]
            res.pose.y = getNextRobotPosition[1]
            res.pose.theta = getNextRobotPosition[2]
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetRobotVelocity(self, req):
        try:
            getRobotVelocity = self.motionProxy.getRobotVelocity()
            res = GetRobotVelocityResponse()
            res.velocity.x = getRobotVelocity[0]
            res.velocity.y = getRobotVelocity[1]
            res.velocity.theta = getRobotVelocity[2]
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetMoveArmsEnabled(self, req):
        try:
            if req.arm.data == 0:
                arm = "LArm"
            elif req.arm.data == 1:
                arm = "RArm"
            else:
                arm = "Arms"
            res = GetArmsEnabledResponse()
            res.status = self.motionProxy.getMoveArmsEnabled(arm)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleSetMoveArmsEnabled(self, req):
        try:
            self.motionProxy.setMoveArmsEnabled(req.left_arm, req.right_arm)
            rospy.loginfo("Left Arm is set to " + str(req.left_arm) + " and Right Arm is set to " + str(req.right_arm))
            return SetArmsEnabledResponse()
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
    locomotionControl = NaoqiLocomotionControl()
    locomotionControl.start()
    rospy.spin()
