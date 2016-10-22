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
from std_msgs.msg import Int64, Int64MultiArray
from naoqi_bridge_msgs.msg import GazeAnalysis
from naoqi_bridge_msgs.srv import (
    SetGazeAnalysisToleranceResponse,
    SetGazeAnalysisTolerance,
    GetGazeAnalysisToleranceResponse,
    GetGazeAnalysisTolerance)

class NaoqiGazeAnalysis (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_gaze_analysis')
        self.connectNaoQi()
        self.gazeAnalysisPub = rospy.Publisher("gaze_analysis", GazeAnalysis, queue_size=10)
        self.personStartsLookingAtRobotPub = rospy.Publisher("person_starts_looking_at_robot", Int64, queue_size=10)
        self.personStopsLookingAtRobotPub = rospy.Publisher("person_stops_looking_at_robot", Int64, queue_size=10)
        self.peopleLookingAtRobotPub = rospy.Publisher("people_looking_at_robot", Int64MultiArray, queue_size=10)
        self.setToleranceSrv = rospy.Service("set_gaze_analysis_tolerance", SetGazeAnalysisTolerance, self.handleSetGazeAnalysisTolerance)
        self.getToleranceSrv = rospy.Service("get_gaze_analysis_tolerance", GetGazeAnalysisTolerance, self.handleGetGazeAnalysisTolerance)
        rospy.loginfo("naoqi_gaze_analysis is initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.gazeProxy = self.get_proxy("ALGazeAnalysis")
        if self.memProxy is None or self.gazeProxy is None:
            exit(1)

    def handleSetGazeAnalysisTolerance(self, req):
        res = SetGazeAnalysisToleranceResponse()
        res.success = False
        try:
            self.gazeProxy.setTolerance(req.tolerance)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleGetGazeAnalysisTolerance(self, req):
        try:
            res = GetGazeAnalysisToleranceResponse()
            res.tolerance = self.gazeProxy.getTolerance()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                # ALMemory Keys
                People_ID = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
                if (len(People_ID)) > 0:
                    for i in range(len(People_ID)):
                        eye_opening_degree_event_name = "PeoplePerception/Person/" + str(People_ID[i]) + "/EyeOpeningDegree"
                        gaze_direction_event_name = "PeoplePerception/Person/" + str(People_ID[i]) + "/GazeDirection"
                        head_angles_event_name = "PeoplePerception/Person/" + str(People_ID[i]) + "/HeadAngles"
                        is_looking_at_robot_event_name = "PeoplePerception/Person/" + str(People_ID[i]) + "/IsLookingAtRobot"
                        looking_at_robot_score_event_name = "PeoplePerception/Person/" + str(People_ID[i]) + "/LookingAtRobotScore"

                        gaze_analysis_msg = GazeAnalysis()
                        gaze_analysis_msg.header.stamp = rospy.get_rostime()
                        gaze_analysis_msg.people_id = People_ID[i]

                        data_list =  self.memProxy.getDataList(eye_opening_degree_event_name)
                        if (len (data_list)) > 0:
                            eye_opening_degree_data = self.memProxy.getData(eye_opening_degree_event_name)
                            if (len (eye_opening_degree_data) > 0):
                                gaze_analysis_msg.left_eye_opening_degree = eye_opening_degree_data[0]
                                gaze_analysis_msg.right_eye_opening_degree = eye_opening_degree_data[1]
                            
                        data_list =  self.memProxy.getDataList(gaze_direction_event_name)
                        if (len (data_list)) > 0:
                            gaze_direction_data = self.memProxy.getData(gaze_direction_event_name)
                            if (len (gaze_direction_data) > 0):
                                gaze_analysis_msg.gaze_direction_yaw = gaze_direction_data[0]
                                gaze_analysis_msg.gaze_direction_pitch = gaze_direction_data[1]

                        data_list =  self.memProxy.getDataList(head_angles_event_name)
                        if (len (data_list)) > 0:
                            head_angles_data = self.memProxy.getData(head_angles_event_name)
                            if (len (head_angles_data) > 0):
                                gaze_analysis_msg.head_angles.x = head_angles_data[0]
                                gaze_analysis_msg.head_angles.y = head_angles_data[1]
                                gaze_analysis_msg.head_angles.z = head_angles_data[2]

                        data_list =  self.memProxy.getDataList(is_looking_at_robot_event_name)
                        if (len (data_list)) > 0:
                           gaze_analysis_msg.is_looking_at_robot = self.memProxy.getData(is_looking_at_robot_event_name)
                           
                        data_list =  self.memProxy.getDataList(looking_at_robot_score_event_name)
                        if (len (data_list)) > 0:
                            gaze_analysis_msg.looking_at_robot_score = self.memProxy.getData(looking_at_robot_score_event_name)
                        
                        self.gazeAnalysisPub.publish(gaze_analysis_msg)
                # GazeAnalysis/PersonStartsLookingAtRobot
                data = self.memProxy.getData("GazeAnalysis/PersonStartsLookingAtRobot")
                person_starts_looking_at_robot_msg = Int64()
                if data != None:
                    person_starts_looking_at_robot_msg.data = data
                    self.personStartsLookingAtRobotPub.publish(person_starts_looking_at_robot_msg)
                # GazeAnalysis/PersonStopsLookingAtRobot
                data = self.memProxy.getData("GazeAnalysis/PersonStopsLookingAtRobot")
                person_stops_looking_at_robot_msg = Int64()
                if data != None:
                    person_stops_looking_at_robot_msg.data = data
                    self.personStopsLookingAtRobotPub.publish(person_stops_looking_at_robot_msg)
                # GazeAnalysis/PeopleLookingAtRobot
                data_list = self.memProxy.getData("GazeAnalysis/PeopleLookingAtRobot")
                people_looking_at_robot_msg = Int64MultiArray()
                if data_list != None and (len(data_list)) > 0:
                    for i in range(len(data_list)):
                        people_looking_at_robot_msg.data.append(data_list[i])
                    self.peopleLookingAtRobotPub.publish(people_looking_at_robot_msg)
                                                                        
            except RuntimeError, e:
                pass
                #print "Error accessing ALMemory, exiting...\n"
                #print e
                #rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    gaze_analysis = NaoqiGazeAnalysis()
    gaze_analysis.start()
    rospy.spin()
