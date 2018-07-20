#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.srv import (
    GetFloatResponse,
    GetFloat,
    SetFloatResponse,
    SetFloat,
    ComputeEngagementZoneBy3DCoordinatesResponse,
    ComputeEngagementZoneBy3DCoordinates,
    ComputeEngagementZoneByAngularAndDistanceResponse,
    ComputeEngagementZoneByAngularAndDistance
)
class NaoqiEngagementZones(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_engagement_zones')
        self.connectNaoQi()
        self.rate = rospy.Rate(10)       
        self.computeEngagementZoneBasedOn3DCoordinatesSrv = rospy.Service("compute_engagement_zone_by_3dcoords", ComputeEngagementZoneBy3DCoordinates, self.handleComputeEngagementZoneBasedOn3DCoordinatesSrv)
        self.computeEngagementZoneBasedOnAngularCoordinatesSrv = rospy.Service("compute_engagement_zone_by_angular_and_distance", ComputeEngagementZoneByAngularAndDistance, self.handleComputeEngagementZoneBasedOnAngularCoordinatesSrv)
        self.getFirstLimitDistanceSrv = rospy.Service("get_first_limit_distance", GetFloat, self.handleGetFirstLimitDistanceSrv)
        self.getLimitAngleSrv = rospy.Service("get_limit_angle", GetFloat, self.handleGetLimitAngleSrv)
        self.getSecondLimitDistanceSrv = rospy.Service("get_second_limit_distance", GetFloat, self.handleGetSecondLimitDistanceSrv)
        self.setFirstLimitDistanceSrv = rospy.Service("set_first_limit_distance", SetFloat, self.handleSetFirstLimitDistanceSrv)
        self.setLimitAngleSrv = rospy.Service("set_limit_angle", SetFloat, self.handleSetLimitAngleSrv)
        self.setSecondLimitDistanceSrv = rospy.Service("set_second_limit_distance", SetFloat, self.handleSetSecondLimitDistanceSrv)
        rospy.loginfo("naoqi_engagement_zones initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.engagementProxy = self.get_proxy("ALEngagementZones")
        if self.engagementProxy is None:
            rospy.logerr("Could not get a proxy to ALEngagementZones")
            exit(1)
        self.memProxy = self.get_proxy("ALMemory")
        if self.memProxy is None:
            rospy.logerr("Could not get a proxy to ALMemory")
            exit(1)
    
    def handleComputeEngagementZoneBasedOn3DCoordinatesSrv(self, req):
        try:
            res = ComputeEngagementZoneBy3DCoordinatesResponse()
            res.engagement_zone = self.engagementProxy.computeEngagementZone(req.coordinate.x, req.coordinate.y, req.coordinate.z)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleComputeEngagementZoneBasedOnAngularCoordinatesSrv(self, req):
        try:
            res = ComputeEngagementZoneByAngularAndDistanceResponse()
            camera_coords = []
            camera_coords.append(req.camera_position_from_robot_frame.linear.x)
            camera_coords.append(req.camera_position_from_robot_frame.linear.y)
            camera_coords.append(req.camera_position_from_robot_frame.linear.z)
            camera_coords.append(req.camera_position_from_robot_frame.angular.x)
            camera_coords.append(req.camera_position_from_robot_frame.angular.y)
            camera_coords.append(req.camera_position_from_robot_frame.angular.z)
            res.engagement_zone = self.engagementProxy.computeEngagementZone(req.x_angle, req.y_angle, req.distance, camera_coords)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetFirstLimitDistanceSrv(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.engagementProxy.getFirstLimitDistance()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleGetLimitAngleSrv(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.engagementProxy.getLimitAngle()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None        

    def handleGetSecondLimitDistanceSrv(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.engagementProxy.getSecondLimitDistance()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetFirstLimitDistanceSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            if req.data > 0.0 and req.data < self.engagementProxy.getSecondLimitDistance():
                self.engagementProxy.setFirstLimitDistance(req.data)
                res.success = True
            else:
                rospy.logerr("The first distance must be positive and smaller than the second distance")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetLimitAngleSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.engagementProxy.setLimitAngle(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetSecondLimitDistanceSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.engagementProxy.setSecondLimitDistance(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

if __name__ == '__main__':
    engagement_zones = NaoqiEngagementZones()
    rospy.spin()
