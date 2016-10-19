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
    Int64,
    Int64MultiArray,
    Bool,)
from naoqi_bridge_msgs.msg import PeopleDetected
from std_srvs.srv import (
    EmptyResponse,
    Empty)
from naoqi_bridge_msgs.srv import (
    GetMaximumDetectionRangeResponse,
    GetMaximumDetectionRange,
    GetTimeBeforePersonDisappearsResponse,
    GetTimeBeforePersonDisappears,
    GetTimeBeforeVisiblePersonDisappearsResponse,
    GetTimeBeforeVisiblePersonDisappears,
    IsFaceDetectionEnabledResponse,
    IsFaceDetectionEnabled,
    IsFastModeEnabledResponse,
    IsFastModeEnabled,
    IsGraphicalDisplayEnabledResponse,
    IsGraphicalDisplayEnabled,
    IsMovementDetectionEnabledResponse,
    IsMovementDetectionEnabled,
    SetFastModeEnabledResponse,
    SetFastModeEnabled,
    SetGraphicalDisplayEnabledResponse,
    SetGraphicalDisplayEnabled,
    SetMaximumDetectionRangeResponse,
    SetMaximumDetectionRange,
    SetMovementDetectionEnabledResponse,
    SetMovementDetectionEnabled,
    SetTimeBeforePersonDisappearsResponse,
    SetTimeBeforePersonDisappears,
    SetTimeBeforeVisiblePersonDisappearsResponse,
    SetTimeBeforeVisiblePersonDisappears
    )

class NaoqiPeoplePerception (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_peoplePerception')
        self.connectNaoQi()
        self.pre_people_detected_list = None
        self.justArrivedPub = rospy.Publisher("PeoplePerception/JustArrived", Int64, queue_size=10)
        self.justLeftPub = rospy.Publisher("PeoplePerception/JustLeft", Int64, queue_size=10)
        self.peopleDetectedPub = rospy.Publisher("PeoplePerception/PeopleDetected", PeopleDetected, queue_size=10)
        self.populationUpdatedPub = rospy.Publisher("PeoplePerception/PopulationUpdated", Bool, queue_size=10)
        self.peopleListPub = rospy.Publisher("PeoplePerception/PeopleList", Int64MultiArray, queue_size=10)
        self.nonVisiblePeopleListPub = rospy.Publisher("PeoplePerception/NonVisiblePeopleList", Int64MultiArray, queue_size=10)
        self.visiblePeopleListPub = rospy.Publisher("PeoplePerception/VisiblePeopleList", Int64MultiArray, queue_size=10)
        self.maximumDetectionRangeUpdatedPub = rospy.Publisher("PeoplePerception/MaximumDetectionRangeUpdated", Bool, queue_size=10)
        
        self.getMaximumDetectionRangeSrv = rospy.Service("get_maximum_detection_range", GetMaximumDetectionRange, self.handleGetMaximumDetectionRange)
        self.getTimeBeforePersonDisappearsSrv = rospy.Service("get_time_before_person_disappears", GetTimeBeforePersonDisappears , self.handleGetTimeBeforePersonDisappears)
        self.getTimeBeforeVisiblePersonDisappearsSrv = rospy.Service("get_time_before_visible_person_disappears", GetTimeBeforeVisiblePersonDisappears , self.handleGetTimeBeforeVisiblePersonDisappears)
        self.isFaceDetectionEnabledSrv = rospy.Service("is_face_detection_enabled", IsFaceDetectionEnabled, self.handleIsFaceDetectionEnabled)
        self.isFastModeEnabledSrv = rospy.Service("get_fast_mode_enabled", IsFastModeEnabled, self.handleIsFastModeEnabled)
        self.isGraphicalDisplayEnabledSrv = rospy.Service("get_graphical_display_enabled", IsGraphicalDisplayEnabled, self.handleIsGraphicalDisplayEnabled)
        self.isMovementDetectionEnabledSrv = rospy.Service("get_movement_detection_enabled", IsMovementDetectionEnabled, self.handleIsMovementDetectionEnabled)
        self.resetPopulationSrv = rospy.Service("reset_population", Empty, self.handleResetPopulation)
        self.setFastModeEnabledSrv = rospy.Service("set_fast_mode_enabled", SetFastModeEnabled, self.handleSetFastModeEnabled)
        self.setGraphicalDisplayEnabledSrv = rospy.Service("set_graphical_display_enabled", SetGraphicalDisplayEnabled, self.handleSetGraphicalDisplayEnabled)
        self.setMaximumDetectionRangeSrv = rospy.Service("set_maximum_detection_range_enabled", SetMaximumDetectionRange, self.handleSetMaximumDetectionRange)
        self.setMovementDetectionEnabledSrv = rospy.Service("set_movement_detection_enabled", SetMovementDetectionEnabled, self.handleSetMovementDetectionEnabled)
        self.setTimeBeforePersonDisappearsSrv = rospy.Service("set_time_before_person_disappears", SetTimeBeforePersonDisappears, self.handleSetTimeBeforePersonDisappears)
        self.setTimeBeforeVisiblePersonDisappearsSrv = rospy.Service("set_time_before_visible_person_disappears", SetTimeBeforeVisiblePersonDisappears, self.handleSetTimeBeforeVisiblePersonDisappears)
        rospy.loginfo("naoqi_peoplePerception is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.peoplePerceptionProxy = self.get_proxy("ALPeoplePerception")
        if self.memProxy is None or self.peoplePerceptionProxy is None:
            exit(1)
            
    def handleGetMaximumDetectionRange(self, req):
        try:
            res = GetMaximumDetectionRangeResponse()
            res.ranges = self.peoplePerceptionProxy.getMaximumDetectionRange()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTimeBeforePersonDisappears(self, req):
        try:
            res = GetTimeBeforePersonDisappearsResponse()
            res.time_before_person_disappears = self.peoplePerceptionProxy.getTimeBeforePersonDisappears()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTimeBeforeVisiblePersonDisappears(self, req):
        try:
            res = GetTimeBeforeVisiblePersonDisappearsResponse()
            res.time_before_visible_person_disappears = self.peoplePerceptionProxy.getTimeBeforeVisiblePersonDisappears()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleIsFaceDetectionEnabled(self, req):
        try:
            res = IsFaceDetectionEnabledResponse()
            res.status = self.peoplePerceptionProxy.isFaceDetectionEnabled()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsFastModeEnabled(self, req):
        try:
            res = IsFastModeEnabledResponse()
            res.status = self.peoplePerceptionProxy.isFastModeEnabled()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsGraphicalDisplayEnabled(self, req):
        try:
            res = IsGraphicalDisplayEnabledResponse()
            res.status = self.peoplePerceptionProxy.isGraphicalDisplayEnabled()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsMovementDetectionEnabled(self, req):
        try:
            res = IsMovementDetectionEnabledResponse()
            res.status = self.peoplePerceptionProxy.isMovementDetectionEnabled()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleResetPopulation(self, req):
        try:
            res = EmptyResponse()
            self.peoplePerceptionProxy.resetPopulation()  
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetFastModeEnabled(self, req):
        res = SetFastModeEnabledResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setFastModeEnabled(req.enable)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetGraphicalDisplayEnabled(self, req):
        res = SetGraphicalDisplayEnabledResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setGraphicalDisplayEnabled(req.enable)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMaximumDetectionRange(self, req):
        res = SetMaximumDetectionRangeResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setMaximumDetectionRange(req.ranges)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMovementDetectionEnabled(self, req):
        res = SetMovementDetectionEnabledResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setMovementDetectionEnabled(req.enable)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTimeBeforePersonDisappears(self, req):
        res = SetTimeBeforePersonDisappearsResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setTimeBeforePersonDisappears(req.seconds)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTimeBeforeVisiblePersonDisappears(self, req):
        res = SetTimeBeforeVisiblePersonDisappearsResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setTimeBeforeVisiblePersonDisappears(req.seconds)  
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                # JustArrived
                just_arrived_msg = Int64()
                just_arrived_msg.data = self.memProxy.getData("PeoplePerception/JustArrived")
                if just_arrived_msg.data != None:
                    self.justArrivedPub.publish(just_arrived_msg)

                # JustLeft
                just_left_msg = Int64()
                just_left_msg.data = self.memProxy.getData("PeoplePerception/JustLeft")
                if just_left_msg.data != None:
                    self.justLeftPub.publish(just_left_msg)

                # PeopleDetected
                people_detected_list = self.memProxy.getData("PeoplePerception/PeopleDetected")
                if people_detected_list != None and (len (people_detected_list) > 0) and people_detected_list[1] != self.pre_people_detected_list:
                    for i in range (len (people_detected_list[1])):
                        people_detected_msg = PeopleDetected()
                        people_detected_msg.header.stamp = rospy.get_rostime()
                        
                        people_detected_msg.people_id = people_detected_list[1][i][0]
                        people_detected_msg.distance = people_detected_list[1][i][1]
                        people_detected_msg.pitch = people_detected_list[1][i][2]
                        people_detected_msg.yaw = people_detected_list[1][i][3]
                        
                        people_detected_msg.camera_pose_in_torso_frame.linear.x = people_detected_list[2][0]
                        people_detected_msg.camera_pose_in_torso_frame.linear.y = people_detected_list[2][1]
                        people_detected_msg.camera_pose_in_torso_frame.linear.z = people_detected_list[2][2]
                        people_detected_msg.camera_pose_in_torso_frame.angular.x = people_detected_list[2][3]
                        people_detected_msg.camera_pose_in_torso_frame.angular.y = people_detected_list[2][4]
                        people_detected_msg.camera_pose_in_torso_frame.angular.z = people_detected_list[2][5]
                        
                        people_detected_msg.camera_pose_in_robot_frame.linear.x = people_detected_list[3][0]
                        people_detected_msg.camera_pose_in_robot_frame.linear.y = people_detected_list[3][1]
                        people_detected_msg.camera_pose_in_robot_frame.linear.z = people_detected_list[3][2]
                        people_detected_msg.camera_pose_in_robot_frame.angular.x = people_detected_list[3][3]
                        people_detected_msg.camera_pose_in_robot_frame.angular.y = people_detected_list[3][4]
                        people_detected_msg.camera_pose_in_robot_frame.angular.z = people_detected_list[3][5]
                        
                        people_detected_msg.camera_id = people_detected_list[4]
    
                        # PositionInTorsoFrame
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInTorsoFrame")
                        if (len (data_list)) > 0:
                            position_in_torso_frame = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInTorsoFrame")
                            if position_in_torso_frame != None:
                                people_detected_msg.head_position_in_torso_frame.x = position_in_torso_frame[0]
                                people_detected_msg.head_position_in_torso_frame.y = position_in_torso_frame[1]
                                people_detected_msg.head_position_in_torso_frame.z = position_in_torso_frame[2]

                        # PositionInRobotFrame
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInRobotFrame")
                        if (len (data_list)) > 0:
                            position_in_robot_frame = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInRobotFrame")
                            if position_in_robot_frame != None:
                                people_detected_msg.head_position_in_robot_frame.x = position_in_robot_frame[0]
                                people_detected_msg.head_position_in_robot_frame.y = position_in_robot_frame[1]
                                people_detected_msg.head_position_in_robot_frame.z = position_in_robot_frame[2]
                        
                        # PositionInWorldFrame
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInWorldFrame")
                        if (len (data_list)) > 0:
                            position_in_world_frame = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInWorldFrame")
                            if position_in_world_frame != None:
                                people_detected_msg.head_position_in_world_frame.x = position_in_world_frame[0]
                                people_detected_msg.head_position_in_world_frame.y = position_in_world_frame[1]
                                people_detected_msg.head_position_in_world_frame.z = position_in_world_frame[2]
                           
                        # IsFaceDetected
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/IsFaceDetected")
                        if (len (data_list)) > 0:
                            is_face_detected = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/IsFaceDetected") 
                            if is_face_detected != None:
                                people_detected_msg.is_face_detected = is_face_detected
                                
                        # IsVisible
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/IsVisible")
                        if (len (data_list)) > 0:
                            is_visible = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/IsVisible")
                            if is_visible != None:
                                people_detected_msg.is_visible = is_visible
                        
                        # NotSeenSince
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/NotSeenSince")
                        if (len (data_list)) > 0:
                            not_seen_since = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/NotSeenSince")
                            if not_seen_since != None:
                                people_detected_msg.not_seen_since = not_seen_since

                        # PresentSince
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PresentSince")
                        if (len (data_list)) > 0:
                            present_since = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PresentSince")
                            if present_since != None:
                                people_detected_msg.present_since = present_since

                        # RealHeight
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/RealHeight")
                        if (len (data_list)) > 0:
                            real_height = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/RealHeight")
                            if real_height != None:
                                people_detected_msg.real_height = real_height
                            
                        # ShirtColor
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ShirtColor")
                        if (len (data_list)) > 0:
                            shirt_color = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ShirtColor")
                            if shirt_color != None:
                                people_detected_msg.shirt_color = shirt_color

                        # ShirtColorHSV
                        data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ShirtColorHSv")
                        if (len (data_list)) > 0:
                            shirt_color_hsv = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ShirtColorHSV")
                            if shirt_color_hsv != None:
                                people_detected_msg.hsv.x = shirt_color_hsv[0]
                                people_detected_msg.hsv.y = shirt_color_hsv[1]
                                people_detected_msg.hsv.z = shirt_color_hsv[2]
                            
                        self.peopleDetectedPub.publish(people_detected_msg)
                    self.pre_people_detected_list = people_detected_list[1]

                # Population Updated
                population_updated_msg = Bool()
                population_updated_msg.data = self.memProxy.getData("PeoplePerception/PopulationUpdated")
                if population_updated_msg.data != None:
                    self.populationUpdatedPub.publish(population_updated_msg)

                # People List
                people_list_msg = Int64MultiArray()
                people_list = self.memProxy.getData("PeoplePerception/PeopleList")
                if (len (people_list)) > 0:
                    for i in range (len (people_list)):
                        people_list_msg.data.append(people_list[i])
                    self.peopleListPub.publish(people_list_msg)
                for i in range(len(people_list)):
                    del people_list_msg.data[0]

                # Non Visible People List
                non_visible_people_list_msg = Int64MultiArray()
                non_visible_people_list = self.memProxy.getData("PeoplePerception/NonVisiblePeopleList")
                if (len (non_visible_people_list)) > 0:
                    for i in range (len (non_visible_people_list)):
                        non_visible_people_list_msg.data.append(non_visible_people_list[i])
                    self.nonVisiblePeopleListPub.publish(non_visible_people_list_msg)
                for i in range(len(non_visible_people_list)):
                    del non_visible_people_list_msg.data[0]

                # Visible People List
                visible_people_list_msg = Int64MultiArray()
                visible_people_list = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
                if (len (visible_people_list)) > 0:
                    for i in range (len (visible_people_list)):
                        visible_people_list_msg.data.append(visible_people_list[i])
                    self.visiblePeopleListPub.publish(visible_people_list_msg)
                for i in range(len(visible_people_list)):
                    del visible_people_list_msg.data[0]

                # Maximum Detection Range Updated
                maximum_detection_range_updated_msg = Bool()
                maximum_detection_range_updated_msg.data = self.memProxy.getData("PeoplePerception/MaximumDetectionRangeUpdated")
                if maximum_detection_range_updated_msg.data != None:
                    self.maximumDetectionRangeUpdatedPub.publish(maximum_detection_range_updated_msg)
                                                
            except RuntimeError, e:
                pass

if __name__ == '__main__':
    peoplePerception = NaoqiPeoplePerception()
    peoplePerception.start()
    rospy.spin()
