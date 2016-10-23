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
from std_msgs.msg import (
    Float64,
    Int64,
    Bool,
    Int64MultiArray)
from naoqi_bridge_msgs.msg import EngagementZoneOfPerson, MovementInfo
from naoqi_bridge_msgs.srv import ( 
    ComputeEngagementZoneBasedOn3DCoordinatesResponse,
    ComputeEngagementZoneBasedOn3DCoordinates,
    ComputeEngagementZoneBasedOnAngularCoordinatesResponse,
    ComputeEngagementZoneBasedOnAngularCoordinates,
    GetFirstLimitDistanceResponse,
    GetFirstLimitDistance,
    GetLimitAngleResponse,
    GetLimitAngle,
    GetSecondLimitDistanceResponse,
    GetSecondLimitDistance,
    SetFirstLimitDistanceResponse,
    SetFirstLimitDistance,
    SetLimitAngleResponse,
    SetLimitAngle,
    SetSecondLimitDistanceResponse,
    SetSecondLimitDistance
)
class NaoqiEngagementZones(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_engagement_zones')
        self.connectNaoQi()
        
        self.computeEngagementZoneBasedOn3DCoordinatesSrv = rospy.Service("compute_engagement_zone_by_3dcoords", ComputeEngagementZoneBasedOn3DCoordinates , self.handleComputeEngagementZoneBasedOn3DCoordinatesSrv)
        self.computeEngagementZoneBasedOnAngularCoordinatesSrv = rospy.Service("compute_engagement_zone_by_angular_coords", ComputeEngagementZoneBasedOnAngularCoordinates , self.handleComputeEngagementZoneBasedOnAngularCoordinatesSrv)
        self.getFirstLimitDistanceSrv = rospy.Service("get_first_limit_distance", GetFirstLimitDistance , self.handleGetFirstLimitDistanceSrv)
        self.getLimitAngleSrv = rospy.Service("get_limit_angle", GetLimitAngle , self.handleGetLimitAngleSrv)
        self.getSecondLimitDistanceSrv = rospy.Service("get_second_limit_distance", GetSecondLimitDistance , self.handleGetSecondLimitDistanceSrv)
        self.setFirstLimitDistanceSrv = rospy.Service("set_first_limit_distance", SetFirstLimitDistance , self.handleSetFirstLimitDistanceSrv)
        self.setLimitAngleSrv = rospy.Service("set_limit_angle", SetLimitAngle , self.handleSetLimitAngleSrv)
        self.setSecondLimitDistanceSrv = rospy.Service("set_second_limit_distance", SetSecondLimitDistance , self.handleSetSecondLimitDistanceSrv)
        self.limitAngleUpdatedPub = rospy.Publisher("limit_angle_updated", Float64 , queue_size=10)
        self.firstLimitDistanceUpdatedPub = rospy.Publisher("first_limit_distance_updated", Float64, queue_size=10)
        self.secondLimitDistanceUpdatedPub = rospy.Publisher("second_limit_distance_updated", Float64, queue_size=10)
        self.peopleInZonesUpdatedPub = rospy.Publisher("people_in_zones_updated", Bool, queue_size=10)
        self.personApproachedPub = rospy.Publisher("person_approached", Int64, queue_size=10)
        self.personMovedAwayPub = rospy.Publisher("person_moved_away", Int64, queue_size=10)
        self.personEnteredZone1Pub = rospy.Publisher("person_entered_zone1", Int64, queue_size=10)
        self.personEnteredZone2Pub = rospy.Publisher("person_entered_zone2", Int64, queue_size=10)
        self.personEnteredZone3Pub = rospy.Publisher("person_entered_zone3", Int64, queue_size=10)
        self.movementsInZonesUpdatedPub = rospy.Publisher("movements_in_zone_updated", Bool, queue_size=10)
        self.engagementZoneByPersonPub = rospy.Publisher("engagement_zone_by_person", EngagementZoneOfPerson, queue_size=10)
        self.peopleInZone1Pub = rospy.Publisher("people_in_zone1", Int64MultiArray, queue_size=10)
        self.peopleInZone2Pub = rospy.Publisher("people_in_zone2", Int64MultiArray, queue_size=10)
        self.peopleInZone3Pub = rospy.Publisher("people_in_zone3", Int64MultiArray, queue_size=10)
        self.lastMovementsInZone1Pub = rospy.Publisher("last_movements_in_zone1", MovementInfo, queue_size=10)
        self.lastMovementsInZone2Pub = rospy.Publisher("last_movements_in_zone2", MovementInfo, queue_size=10)
        self.lastMovementsInZone3Pub = rospy.Publisher("last_movements_in_zone3", MovementInfo, queue_size=10) 
        rospy.loginfo("naoqi_engagement_zones initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.engagementProxy = self.get_proxy("ALEngagementZones")
        self.peopleProxy = self.get_proxy("ALPeoplePerception")
        self.moveDetectionProxy = self.get_proxy("ALMovementDetection")
        if self.memProxy is None or self.engagementProxy is None or self.peopleProxy is None or self.moveDetectionProxy is None:
            exit(1)
    
    def handleComputeEngagementZoneBasedOn3DCoordinatesSrv(self, req):
        try:
            res = ComputeEngagementZoneBasedOn3DCoordinatesResponse()
            res.engagement_zone = self.engagementProxy.computeEngagementZone(req.coordinate.x, req.coordinate.y, req.coordinate.z)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleComputeEngagementZoneBasedOnAngularCoordinatesSrv(self, req):
        try:
            res = ComputeEngagementZoneBasedOnAngularCoordinatesResponse()
            camera_coords = []
            camera_coords.append(req.camera_position_from_frame_robot.linear.x)
            camera_coords.append(req.camera_position_from_frame_robot.linear.y)
            camera_coords.append(req.camera_position_from_frame_robot.linear.z)
            camera_coords.append(req.camera_position_from_frame_robot.angular.x)
            camera_coords.append(req.camera_position_from_frame_robot.angular.y)
            camera_coords.append(req.camera_position_from_frame_robot.angular.z)
            res.engagement_zone = self.engagementProxy.computeEngagementZone(req.x_angle, req.y_angle, req.distance, camera_coords)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetFirstLimitDistanceSrv(self, req):
        try:
            res = GetFirstLimitDistanceResponse()
            res.distance = self.engagementProxy.getFirstLimitDistance()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleGetLimitAngleSrv(self, req):
        try:
            res = GetLimitAngleResponse()
            res.angle = self.engagementProxy.getLimitAngle()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None        

    def handleGetSecondLimitDistanceSrv(self, req):
        try:
            res = GetSecondLimitDistanceResponse()
            res.distance = self.engagementProxy.getSecondLimitDistance()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetFirstLimitDistanceSrv(self, req):
        res = SetFirstLimitDistanceResponse()
        res.success = False
        try:
            self.engagementProxy.setFirstLimitDistance(req.distance)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetLimitAngleSrv(self, req):
        res = SetLimitAngleResponse()
        res.success = False
        try:
            self.engagementProxy.setLimitAngle(req.angle)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetSecondLimitDistanceSrv(self, req):
        res = SetSecondLimitDistanceResponse()
        res.success = False
        try:
            self.engagementProxy.setSecondLimitDistance(req.distance)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                # EngagementZones/LimitAngleUpdated
                limit_angle_updated_msg = Float64()
                limit_angle_updated_msg.data = self.memProxy.getData("EngagementZones/LimitAngleUpdated")
                if limit_angle_updated_msg.data != None:
                    self.limitAngleUpdatedPub.publish(limit_angle_updated_msg)

                # "EngagementZones/FirstLimitDistanceUpdated"
                first_limit_distance_updated_msg = Float64()
                first_limit_distance_updated_msg.data = self.memProxy.getData("EngagementZones/FirstLimitDistanceUpdated")
                if first_limit_distance_updated_msg.data != None:
                    self.firstLimitDistanceUpdatedPub.publish(first_limit_distance_updated_msg)
                
                # "EngagementZones/SecondLimitDistanceUpdated"
                second_limit_distance_updated_msg = Float64()
                second_limit_distance_updated_msg.data = self.memProxy.getData("EngagementZones/SecondLimitDistanceUpdated")
                if second_limit_distance_updated_msg.data != None:
                    self.secondLimitDistanceUpdatedPub.publish(second_limit_distance_updated_msg)

                # "EngagementZones/PeopleInZonesUpdated"
                people_in_zones_updated_msg = Bool()
                data_list = self.memProxy.getDataList("EngagementZones/PeopleInZonesUpdated")
                if (len(data_list) > 0):
                    people_in_zones_updated_msg.data = True
                    self.peopleInZonesUpdatedPub.publish(people_in_zones_updated_msg)
                
                # "EngagementZones/PersonApproached"
                person_approached_msg = Int64()
                person_approached_msg.data = self.memProxy.getData("EngagementZones/PersonApproached")
                if person_approached_msg.data != None:
                    self.personApproachedPub.publish(person_approached_msg)

                # "EngagementZones/PersonMovedAway"
                person_moved_away_msg = Int64()
                person_moved_away_msg.data = self.memProxy.getData("EngagementZones/PersonMovedAway")
                if person_moved_away_msg.data != None:
                    self.personMovedAwayPub.publish(person_moved_away_msg)

                # "EngagementZones/PersonEnteredZone1"
                person_entered_zone1_msg = Int64()
                person_entered_zone1_msg.data = self.memProxy.getData("EngagementZones/PersonEnteredZone1")
                if person_entered_zone1_msg.data != None:
                    self.personEnteredZone1Pub.publish(person_entered_zone1_msg)

                # "EngagementZones/PersonEnteredZone2"
                person_entered_zone2_msg = Int64()
                person_entered_zone2_msg.data = self.memProxy.getData("EngagementZones/PersonEnteredZone2")
                if person_entered_zone2_msg.data != None:
                    self.personEnteredZone2Pub.publish(person_entered_zone2_msg)

                # "EngagementZones/PersonEnteredZone3"
                person_entered_zone3_msg = Int64()
                person_entered_zone3_msg.data = self.memProxy.getData("EngagementZones/PersonEnteredZone3")
                if person_entered_zone3_msg.data != None:
                    self.personEnteredZone3Pub.publish(person_entered_zone3_msg)

                # "EngagementZones/MovementsInZonesUpdated"
                movements_in_zones_updated_msg = Bool()
                data_list = self.memProxy.getDataList("EngagementZones/MovementsInZonesUpdated")
                if (len(data_list) > 0):
                    movements_in_zones_updated_msg.data = True
                    self.movementsInZonesUpdatedPub.publish(movements_in_zones_updated_msg)
                
                # "PeoplePerception/Person/<ID>/EngagementZone"
                People_ID = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
                if (len(People_ID)) > 0:
                    for i in range(len(People_ID)):
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(People_ID[i]) + "/EngagementZone")
                        if (len(data_list) > 0):
                            engagement_zone_by_person_msg = EngagementZoneOfPerson()
                            engagement_zone_by_person_msg.engagement_zone = self.memProxy.getData("PeoplePerception/Person/" + str(People_ID[i]) + "/EngagementZone")
                            engagement_zone_by_person_msg.people_id = People_ID[i]
                            self.engagementZoneByPersonPub.publish(engagement_zone_by_person_msg)
                        
                # "EngagementZones/PeopleInZone1"
                people_in_zone1_msg = Int64MultiArray()
                people_in_zone1_data = self.memProxy.getData("EngagementZones/PeopleInZone1")
                if (len(people_in_zone1_data) > 0):
                    for i in range (len(people_in_zone1_data)):
                        people_in_zone1_msg.data.append(people_in_zone1_data[i])
                    self.peopleInZone1Pub.publish(people_in_zone1_msg)

                # "EngagementZones/PeopleInZone2"
                people_in_zone2_msg = Int64MultiArray()
                people_in_zone2_data = self.memProxy.getData("EngagementZones/PeopleInZone2")
                if (len(people_in_zone2_data) > 0):
                    for i in range (len(people_in_zone2_data)):
                        people_in_zone2_msg.data.append(people_in_zone2_data[i])
                    self.peopleInZone2Pub.publish(people_in_zone2_msg)

                # "EngagementZones/PeopleInZone3"
                people_in_zone3_msg = Int64MultiArray()
                people_in_zone3_data = self.memProxy.getData("EngagementZones/PeopleInZone3")
                if (len(people_in_zone3_data) > 0):
                    for i in range (len(people_in_zone3_data)):
                        people_in_zone3_msg.data.append(people_in_zone3_data[i])
                    self.peopleInZone3Pub.publish(people_in_zone3_msg)

                # EngagementZones/LastMovementsInZone1
                last_movements_in_zone1_msg = MovementInfo()
                last_movements_in_zone1_data = self.memProxy.getData("EngagementZones/LastMovementsInZone1")
                if (len(last_movements_in_zone1_data) > 0): 
                    for i in range(len(last_movements_in_zone1_data[1])):
                        last_movements_in_zone1_msg.header.stamp = rospy.get_rostime()
                        last_movements_in_zone1_msg.camera_pose_in_torso_frame.linear.x = last_movements_in_zone1_data[2][0]
                        last_movements_in_zone1_msg.camera_pose_in_torso_frame.linear.y = last_movements_in_zone1_data[2][1]
                        last_movements_in_zone1_msg.camera_pose_in_torso_frame.linear.z = last_movements_in_zone1_data[2][2]
                        last_movements_in_zone1_msg.camera_pose_in_torso_frame.angular.x = last_movements_in_zone1_data[2][3]
                        last_movements_in_zone1_msg.camera_pose_in_torso_frame.angular.y = last_movements_in_zone1_data[2][4]
                        last_movements_in_zone1_msg.camera_pose_in_torso_frame.angular.z = last_movements_in_zone1_data[2][5]
                        last_movements_in_zone1_msg.camera_pose_in_robot_frame.linear.x = last_movements_in_zone1_data[3][0]
                        last_movements_in_zone1_msg.camera_pose_in_robot_frame.linear.y = last_movements_in_zone1_data[3][1]
                        last_movements_in_zone1_msg.camera_pose_in_robot_frame.linear.z = last_movements_in_zone1_data[3][2]
                        last_movements_in_zone1_msg.camera_pose_in_robot_frame.angular.x = last_movements_in_zone1_data[3][3]
                        last_movements_in_zone1_msg.camera_pose_in_robot_frame.angular.y = last_movements_in_zone1_data[3][4]
                        last_movements_in_zone1_msg.camera_pose_in_robot_frame.angular.z = last_movements_in_zone1_data[3][5]
                        last_movements_in_zone1_msg.camera_id = last_movements_in_zone1_data[4]
                        last_movements_in_zone1_msg.position_of_cog.x = last_movements_in_zone1_data[1][i][0][0]
                        last_movements_in_zone1_msg.position_of_cog.y = last_movements_in_zone1_data[1][i][0][1]
                        last_movements_in_zone1_msg.angular_roi.x = last_movements_in_zone1_data[1][i][1][0]
                        last_movements_in_zone1_msg.angular_roi.y = last_movements_in_zone1_data[1][i][1][1]
                        last_movements_in_zone1_msg.angular_roi.width = last_movements_in_zone1_data[1][i][1][2]
                        last_movements_in_zone1_msg.angular_roi.height = last_movements_in_zone1_data[1][i][1][3]
                        last_movements_in_zone1_msg.proportion_moving_pixels.in_frame = last_movements_in_zone1_data[1][i][2][0]
                        last_movements_in_zone1_msg.proportion_moving_pixels.in_roi = last_movements_in_zone1_data[1][i][2][1]
                        
                        if last_movements_in_zone1_msg.camera_id == 2:
                            last_movements_in_zone1_msg.mean_distance = last_movements_in_zone1_data[1][i][3]
                            last_movements_in_zone1_msg.real_size_roi.real_width = last_movements_in_zone1_data[1][i][4][0]
                            last_movements_in_zone1_msg.real_size_roi.real_height = last_movements_in_zone1_data[1][i][4][1]
                            last_movements_in_zone1_msg.position_of_associated_point.x = last_movements_in_zone1_data[1][i][5][0]
                            last_movements_in_zone1_msg.position_of_associated_point.y = last_movements_in_zone1_data[1][i][5][1]                    
                        self.lastMovementsInZone1Pub.publish(last_movements_in_zone1_msg)

                # EngagementZones/LastMovementsInZone2
                last_movements_in_zone2_msg = MovementInfo()
                last_movements_in_zone2_data = self.memProxy.getData("EngagementZones/LastMovementsInZone2")
                if (len(last_movements_in_zone2_data) > 0): 
                    for i in range(len(last_movements_in_zone2_data[1])):
                        last_movements_in_zone2_msg.header.stamp = rospy.get_rostime()
                        last_movements_in_zone2_msg.camera_pose_in_torso_frame.linear.x = last_movements_in_zone2_data[2][0]
                        last_movements_in_zone2_msg.camera_pose_in_torso_frame.linear.y = last_movements_in_zone2_data[2][1]
                        last_movements_in_zone2_msg.camera_pose_in_torso_frame.linear.z = last_movements_in_zone2_data[2][2]
                        last_movements_in_zone2_msg.camera_pose_in_torso_frame.angular.x = last_movements_in_zone2_data[2][3]
                        last_movements_in_zone2_msg.camera_pose_in_torso_frame.angular.y = last_movements_in_zone2_data[2][4]
                        last_movements_in_zone2_msg.camera_pose_in_torso_frame.angular.z = last_movements_in_zone2_data[2][5]
                        last_movements_in_zone2_msg.camera_pose_in_robot_frame.linear.x = last_movements_in_zone2_data[3][0]
                        last_movements_in_zone2_msg.camera_pose_in_robot_frame.linear.y = last_movements_in_zone2_data[3][1]
                        last_movements_in_zone2_msg.camera_pose_in_robot_frame.linear.z = last_movements_in_zone2_data[3][2]
                        last_movements_in_zone2_msg.camera_pose_in_robot_frame.angular.x = last_movements_in_zone2_data[3][3]
                        last_movements_in_zone2_msg.camera_pose_in_robot_frame.angular.y = last_movements_in_zone2_data[3][4]
                        last_movements_in_zone2_msg.camera_pose_in_robot_frame.angular.z = last_movements_in_zone2_data[3][5]
                        last_movements_in_zone2_msg.camera_id = last_movements_in_zone2_data[4]
                        last_movements_in_zone2_msg.position_of_cog.x = last_movements_in_zone2_data[1][i][0][0]
                        last_movements_in_zone2_msg.position_of_cog.y = last_movements_in_zone2_data[1][i][0][1]
                        last_movements_in_zone2_msg.angular_roi.x = last_movements_in_zone2_data[1][i][1][0]
                        last_movements_in_zone2_msg.angular_roi.y = last_movements_in_zone2_data[1][i][1][1]
                        last_movements_in_zone2_msg.angular_roi.width = last_movements_in_zone2_data[1][i][1][2]
                        last_movements_in_zone2_msg.angular_roi.height = last_movements_in_zone2_data[1][i][1][3]
                        last_movements_in_zone2_msg.proportion_moving_pixels.in_frame = last_movements_in_zone2_data[1][i][2][0]
                        last_movements_in_zone2_msg.proportion_moving_pixels.in_roi = last_movements_in_zone2_data[1][i][2][1]
                    
                        if last_movements_in_zone2_msg.camera_id == 2:
                            last_movements_in_zone2_msg.mean_distance = last_movements_in_zone2_data[1][i][3]
                            last_movements_in_zone2_msg.real_size_roi.real_width = last_movements_in_zone2_data[1][i][4][0]
                            last_movements_in_zone2_msg.real_size_roi.real_height = last_movements_in_zone2_data[1][i][4][1]
                            last_movements_in_zone2_msg.position_of_associated_point.x = last_movements_in_zone2_data[1][i][5][0]
                            last_movements_in_zone2_msg.position_of_associated_point.y = last_movements_in_zone2_data[1][i][5][1]                    
                        self.lastMovementsInZone2Pub.publish(last_movements_in_zone2_msg)

                # EngagementZones/LastMovementsInZone3
                last_movements_in_zone3_msg = MovementInfo()
                last_movements_in_zone3_data = self.memProxy.getData("EngagementZones/LastMovementsInZone3")
                if (len(last_movements_in_zone3_data) > 0): 
                    for i in range(len(last_movements_in_zone3_data[1])):
                        last_movements_in_zone3_msg.header.stamp = rospy.get_rostime()
                        last_movements_in_zone3_msg.camera_pose_in_torso_frame.linear.x = last_movements_in_zone3_data[2][0]
                        last_movements_in_zone3_msg.camera_pose_in_torso_frame.linear.y = last_movements_in_zone3_data[2][1]
                        last_movements_in_zone3_msg.camera_pose_in_torso_frame.linear.z = last_movements_in_zone3_data[2][2]
                        last_movements_in_zone3_msg.camera_pose_in_torso_frame.angular.x = last_movements_in_zone3_data[2][3]
                        last_movements_in_zone3_msg.camera_pose_in_torso_frame.angular.y = last_movements_in_zone3_data[2][4]
                        last_movements_in_zone3_msg.camera_pose_in_torso_frame.angular.z = last_movements_in_zone3_data[2][5]
                        last_movements_in_zone3_msg.camera_pose_in_robot_frame.linear.x = last_movements_in_zone3_data[3][0]
                        last_movements_in_zone3_msg.camera_pose_in_robot_frame.linear.y = last_movements_in_zone3_data[3][1]
                        last_movements_in_zone3_msg.camera_pose_in_robot_frame.linear.z = last_movements_in_zone3_data[3][2]
                        last_movements_in_zone3_msg.camera_pose_in_robot_frame.angular.x = last_movements_in_zone3_data[3][3]
                        last_movements_in_zone3_msg.camera_pose_in_robot_frame.angular.y = last_movements_in_zone3_data[3][4]
                        last_movements_in_zone3_msg.camera_pose_in_robot_frame.angular.z = last_movements_in_zone3_data[3][5]
                        last_movements_in_zone3_msg.camera_id = last_movements_in_zone3_data[4]
                        last_movements_in_zone3_msg.position_of_cog.x = last_movements_in_zone3_data[1][i][0][0]
                        last_movements_in_zone3_msg.position_of_cog.y = last_movements_in_zone3_data[1][i][0][1]
                        last_movements_in_zone3_msg.angular_roi.x = last_movements_in_zone3_data[1][i][1][0]
                        last_movements_in_zone3_msg.angular_roi.y = last_movements_in_zone3_data[1][i][1][1]
                        last_movements_in_zone3_msg.angular_roi.width = last_movements_in_zone3_data[1][i][1][2]
                        last_movements_in_zone3_msg.angular_roi.height = last_movements_in_zone3_data[1][i][1][3]
                        last_movements_in_zone3_msg.proportion_moving_pixels.in_frame = last_movements_in_zone3_data[1][i][2][0]
                        last_movements_in_zone3_msg.proportion_moving_pixels.in_roi = last_movements_in_zone3_data[1][i][2][1]
                        
                        if last_movements_in_zone3_msg.camera_id == 2:
                            last_movements_in_zone3_msg.mean_distance = last_movements_in_zone3_data[1][i][3]
                            last_movements_in_zone3_msg.real_size_roi.real_width = last_movements_in_zone3_data[1][i][4][0]
                            last_movements_in_zone3_msg.real_size_roi.real_height = last_movements_in_zone3_data[1][i][4][1]
                            last_movements_in_zone3_msg.position_of_associated_point.x = last_movements_in_zone3_data[1][i][5][0]
                            last_movements_in_zone3_msg.position_of_associated_point.y = last_movements_in_zone3_data[1][i][5][1]                    
                        self.lastMovementsInZone3Pub.publish(last_movements_in_zone3_msg)

                        
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    engagement_zones = NaoqiEngagementZones()
    engagement_zones.start()
    rospy.spin()
