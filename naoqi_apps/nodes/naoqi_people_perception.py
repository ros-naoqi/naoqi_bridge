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
from naoqi_bridge_msgs.msg import PeopleDetected

class NaoqiPeoplePerception (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_people_perception')
        self.connectNaoQi()
        self.pre_people_detected_list = None
        self.peopleDetectedPub = rospy.Publisher("people_perception/people_detected", PeopleDetected, queue_size=10)        
        rospy.loginfo("naoqi_peoplePerception is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.peoplePerceptionProxy = self.get_proxy("ALPeoplePerception")
        if self.memProxy is None or self.peoplePerceptionProxy is None:
            exit(1)
            
    def run(self):
        while self.is_looping():
            try:
                # PeopleDetected
                if self.peopleDetectedPub.get_num_connections() > 0:
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
                            data_list = self.memProxy.getDataList("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ShirtColorHSV")
                            if (len (data_list)) > 0:
                                shirt_color_hsv = self.memProxy.getData("PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ShirtColorHSV")
                                if shirt_color_hsv != None:
                                    people_detected_msg.hsv.x = shirt_color_hsv[0]
                                    people_detected_msg.hsv.y = shirt_color_hsv[1]
                                    people_detected_msg.hsv.z = shirt_color_hsv[2]
                            
                            self.peopleDetectedPub.publish(people_detected_msg)
                        self.pre_people_detected_list = people_detected_list[1]

            except RuntimeError, e:
                pass

if __name__ == '__main__':
    peoplePerception = NaoqiPeoplePerception()
    peoplePerception.start()
    rospy.spin()
