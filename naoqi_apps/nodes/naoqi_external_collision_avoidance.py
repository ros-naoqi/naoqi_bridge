#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.msg import (
    MoveFailed, 
    ChainVelocityClipped)
from naoqi_bridge_msgs.srv import ( 
    GetFloatResponse,
    GetFloat,
    SetFloatResponse,
    SetFloat,
    GetChainClosestObstaclePositionResponse,
    GetChainClosestObstaclePosition,
    SetExternalCollisionProtectionEnabledResponse,
    SetExternalCollisionProtectionEnabled,
    GetExternalCollisionProtectionEnabledResponse,
    GetExternalCollisionProtectionEnabled,)

class NaoqiExternalCollisionAvoidance(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_external_collision_avoidance')
        self.connectNaoQi()

        self.rate = rospy.Rate(10)
        
        self.moveFailed = MoveFailed()
        self.previousMoveFailed = ""
        self.chainVelocityClipped = ChainVelocityClipped()
        self.previousChainVelocityClipped = ""
        self.getChainClosesObstaclePositionSrv = rospy.Service("get_chain_closest_obstacle_position", GetChainClosestObstaclePosition , self.handleGetChainClosestObstaclePosition) 
        self.getExternalCollisionProtectionEnabledSrv = rospy.Service("get_external_collision_protection_status", GetExternalCollisionProtectionEnabled, self.handleGetExternalCollisionProtectionEnabled)
        self.getOrthogonalSecurityDistanceSrv = rospy.Service("get_orthogonal_distance", GetFloat, self.handleGetOrthogonalSecurityDistance)
        self.getTangentialSecurityDistanceSrv = rospy.Service("get_tangential_distance", GetFloat, self.handleGetTangentialSecurityDistance)
        self.setExternalCollisionProtectionEnabledSrv = rospy.Service("set_external_collision_protection_status", SetExternalCollisionProtectionEnabled, self.handleSetExternalCollisionProtectionEnabled)
        self.setOrthogonalSecurityDistanceSrv = rospy.Service("set_orthogonal_distance", SetFloat, self.handleSetOrthogonalSecurityDistance)
        self.setTangentialSecurityDistanceSrv = rospy.Service("set_tangential_distance", SetFloat, self.handleSetTangentialSecurityDistance)
        self.chainVelocityClippedPub = rospy.Publisher("chain_velocity_clipped", ChainVelocityClipped, queue_size=10) 
        self.moveFailedPub = rospy.Publisher("move_failed", MoveFailed, queue_size=10)       
        rospy.loginfo("naoqi_external_collision_avoidance initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.motionProxy = self.get_proxy("ALMotion")
        if self.memProxy is None:
            rospy.logerr("Could not get a proxy to ALMemory")
            exit(1)
        if self.motionProxy is None:
            rospy.logerr("Could not get a proxy to ALMotion")
            exit(1)
        
    def handleGetChainClosestObstaclePosition(self, req):
        try:
            if req.arm.data == req.arm.LARM:
                arm="LArm"
            elif req.arm.data == req.arm.RARM:
                arm="RArm"
            else:
                rospy.logerr("Arm type should be LArm or RArm.")
                return None
            if not (req.frame.data == req.frame.FRAME_TORSO or req.frame.data == req.frame.FRAME_ROBOT or req.frame.data == req.frame.FRAME_WORLD):
                rospy.logerr("Frame type should be FRAME_TORSO or FRAME_ROBOT or FRAME_WORLD.")
                return None

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
            res = SetFloatResponse()
            if (req.data > 0.001 or req.data == 0.001):
                self.motionProxy.setOrthogonalSecurityDistance(req.data)
                rospy.loginfo("Orthogonal security distance is set to " + str(req.data))
                res.success = True
            else:
                rospy.loginfo("Orthogonal security distance should be 0.001, or more than 0.001")
                res.success = False
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleSetTangentialSecurityDistance(self, req):
        try:
            res = SetFloatResponse()
            if (req.data > 0.001 or req.data == 0.001):
                self.motionProxy.setTangentialSecurityDistance(req.data)
                rospy.loginfo("Tangential security distance is set to " + str(req.data))
                res.success = True
            else:
                rospy.loginfo("Tangential distance should be 0.001, or more than 0.001")
                res.success = False
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleGetOrthogonalSecurityDistance(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.motionProxy.getOrthogonalSecurityDistance()
            rospy.loginfo("Current Orthogonal Security Distance: " + str(res.data))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTangentialSecurityDistance(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.motionProxy.getTangentialSecurityDistance()
            rospy.loginfo("Current Tangential Security Distance: " + str(res.data))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetExternalCollisionProtectionEnabled(self, req):
        res = SetExternalCollisionProtectionEnabledResponse()
        res.success = False
        try:
            if req.name.data == req.name.ALL:
                target_name = "All"
            elif req.name.data == req.name.MOVE:
                target_name = "Move"
            elif req.name.data == req.name.ARMS:
                target_name = "Arms"
            elif req.name.data == req.name.LARM:
                target_name = "LArm"
            elif req.name.data == req.name.RARM:
                target_name = "RArm"
            self.motionProxy.setExternalCollisionProtectionEnabled(target_name, req.status)
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
            if req.name.data == req.name.ALL:
                target_name = "All"
            if req.name.data == req.name.MOVE:
                target_name = "Move"
            if req.name.data == req.name.ARMS:
                target_name = "Arms"
            if req.name.data == req.name.LARM:
                target_name = "LArm"
            if req.name.data == req.name.RARM:
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
                if self.moveFailedPub.get_num_connections() > 0:
                    data_list = self.memProxy.getDataList("ALMotion/MoveFailed")
                    if (len (data_list)) > 0:
                        moveFailedData = self.memProxy.getData("ALMotion/MoveFailed")
                        if moveFailedData != None and moveFailedData != self.previousMoveFailed:
                            self.moveFailed.header.stamp = rospy.get_rostime()
                            self.moveFailed.header.frame_id = "FRAME_WORLD"
                            self.moveFailed.cause = moveFailedData[0]
                            self.moveFailed.status = moveFailedData[1]
                            if moveFailedData[2] and len(moveFailedData[2]) == 3:
                                self.moveFailed.position.x = (moveFailedData[2])[0]
                                self.moveFailed.position.y = (moveFailedData[2])[1]
                                self.moveFailed.position.z = (moveFailedData[2])[2]
                            self.moveFailedPub.publish(self.moveFailed)
                            self.previousMoveFailed = moveFailedData
                if self.chainVelocityClippedPub.get_num_connections() > 0:
                    data_list = self.memProxy.getDataList("ALMotion/Safety/ChainVelocityClipped")
                    if (len (data_list)) > 0:
                        chainVelocityClippedData = self.memProxy.getData("ALMotion/Safety/ChainVelocityClipped")
                        if chainVelocityClippedData != None and chainVelocityClippedData != self.previousChainVelocityClipped:
                            for i in range(len(chainVelocityClippedData)):
                                self.chainVelocityClipped.header.stamp = rospy.get_rostime()
                                self.chainVelocityClipped.header.frame_id = "FRAME_WORLD"
                                if (chainVelocityClippedData[i])[0] == "Head":
                                    self.chainVelocityClipped.chain.data = self.chainVelocityClipped.chain.HEAD
                                elif (chainVelocityClippedData[i])[0] == "Leg":
                                    self.chainVelocityClipped.chain.data = self.chainVelocityClipped.chain.LEG
                                elif (chainVelocityClippedData[i])[0] == "LArm":
                                    self.chainVelocityClipped.chain.data = self.chainVelocityClipped.chain.LARM
                                elif (chainVelocityClippedData[i])[0] == "RArm":
                                    self.chainVelocityClipped.chain.data = self.chainVelocityClipped.chain.RARM
                                elif (chainVelocityClippedData[i])[0] == "LLeg":
                                    self.chainVelocityClipped.chain.data = self.chainVelocityClipped.chain.LLEG
                                elif (chainVelocityClippedData[i])[0] == "RLeg":
                                    self.chainVelocityClipped.chain.data = self.chainVelocityClipped.chain.RLEG
                                self.chainVelocityClipped.obstacle_position.x = ((chainVelocityClippedData[i])[1])[0]
                                self.chainVelocityClipped.obstacle_position.y = ((chainVelocityClippedData[i])[1])[1]
                                self.chainVelocityClipped.obstacle_position.z = ((chainVelocityClippedData[i])[1])[2]
                                self.chainVelocityClippedPub.publish(self.chainVelocityClipped)
                            self.previousChainVelocityClipped = chainVelocityClippedData
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

            self.rate.sleep()

if __name__ == '__main__':
    externalCollisionAvoidance = NaoqiExternalCollisionAvoidance()
    externalCollisionAvoidance.start()
    rospy.spin()
