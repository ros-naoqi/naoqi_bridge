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
from naoqi_bridge_msgs.msg import PeoplePerception

class NaoqiPeoplePerception (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_people_perception')
        self.connectNaoQi()
        self.pre_people_detected_list = None
        self.peoplePerceptionPub = rospy.Publisher("people_perception", PeoplePerception, queue_size=10)
        rospy.loginfo("naoqi_people_perception is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        if self.memProxy is None:
            exit(1)
            
    def run(self):
        while self.is_looping():
            try:
                # PeopleDetected
                if self.peoplePerceptionPub.get_num_connections() > 0:
                    people_detected_list = self.memProxy.getData("PeoplePerception/PeopleDetected")
                    if people_detected_list != None and (len (people_detected_list) > 0) and people_detected_list[1] != self.pre_people_detected_list:
                        people_perception_msg = PeoplePerception()
                        people_perception_msg.header.stamp = rospy.get_rostime()
                        for i in range (len (people_detected_list[1])):
                            people_detected_msg = people_perception_msg.people_detected

                            # PeopleID
                            people_detected_msg.people_id = people_detected_list[1][i][0]
                            # Distance
                            people_detected_msg.distance = people_detected_list[1][i][1]
                            # Pitch
                            people_detected_msg.pitch = people_detected_list[1][i][2]
                            # Yaw
                            people_detected_msg.yaw = people_detected_list[1][i][3]
                        
                            # CameraPoseInTorsoFrame 
                            people_detected_msg.camera_pose_in_torso_frame.linear.x = people_detected_list[2][0]
                            people_detected_msg.camera_pose_in_torso_frame.linear.y = people_detected_list[2][1]
                            people_detected_msg.camera_pose_in_torso_frame.linear.z = people_detected_list[2][2]
                            people_detected_msg.camera_pose_in_torso_frame.angular.x = people_detected_list[2][3]
                            people_detected_msg.camera_pose_in_torso_frame.angular.y = people_detected_list[2][4]
                            people_detected_msg.camera_pose_in_torso_frame.angular.z = people_detected_list[2][5]
                        
                            # CameraPoseInRobotFrame
                            people_detected_msg.camera_pose_in_robot_frame.linear.x = people_detected_list[3][0]
                            people_detected_msg.camera_pose_in_robot_frame.linear.y = people_detected_list[3][1]
                            people_detected_msg.camera_pose_in_robot_frame.linear.z = people_detected_list[3][2]
                            people_detected_msg.camera_pose_in_robot_frame.angular.x = people_detected_list[3][3]
                            people_detected_msg.camera_pose_in_robot_frame.angular.y = people_detected_list[3][4]
                            people_detected_msg.camera_pose_in_robot_frame.angular.z = people_detected_list[3][5]
                        
                            # CameraID
                            people_detected_msg.camera_id = people_detected_list[4]

                            position_in_torso_frame = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInTorsoFrame"
                            position_in_robot_frame = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInRobotFrame"
                            position_in_world_frame = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PositionInWorldFrame"
                            is_face_detected = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/IsFaceDetected"
                            is_visible = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/IsVisible"
                            not_seen_since = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/NotSeenSince"
                            present_since = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/PresentSince"
                            real_height = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/RealHeight"
                            shirt_color = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ShirtColor"
                            shirt_color_hsv = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ShirtColorHSV"

                            # PositionInTorsoFrame
                            data_list = self.memProxy.getDataList(position_in_torso_frame)
                            if (len (data_list)) > 0:
                                position_in_torso_frame_data = self.memProxy.getData(position_in_torso_frame)
                                if position_in_torso_frame_data != None:
                                    people_detected_msg.head_position_in_torso_frame.x = position_in_torso_frame_data[0]
                                    people_detected_msg.head_position_in_torso_frame.y = position_in_torso_frame_data[1]
                                    people_detected_msg.head_position_in_torso_frame.z = position_in_torso_frame_data[2]

                            # PositionInRobotFrame
                            data_list = self.memProxy.getDataList(position_in_robot_frame)
                            if (len (data_list)) > 0:
                                position_in_robot_frame_data = self.memProxy.getData(position_in_robot_frame)
                                if position_in_robot_frame_data != None:
                                    people_detected_msg.head_position_in_robot_frame.x = position_in_robot_frame_data[0]
                                    people_detected_msg.head_position_in_robot_frame.y = position_in_robot_frame_data[1]
                                    people_detected_msg.head_position_in_robot_frame.z = position_in_robot_frame_data[2]
                        
                            # PositionInWorldFrame
                            data_list = self.memProxy.getDataList(position_in_world_frame)
                            if (len (data_list)) > 0:
                                position_in_world_frame_data = self.memProxy.getData(position_in_world_frame)
                                if position_in_world_frame_data != None:
                                    people_detected_msg.head_position_in_world_frame.x = position_in_world_frame_data[0]
                                    people_detected_msg.head_position_in_world_frame.y = position_in_world_frame_data[1]
                                    people_detected_msg.head_position_in_world_frame.z = position_in_world_frame_data[2]
                                    
                            # IsFaceDetected
                            data_list = self.memProxy.getDataList(is_face_detected)
                            if (len (data_list)) > 0:
                                is_face_detected_data = self.memProxy.getData(is_face_detected) 
                                if is_face_detected_data != None:
                                    people_detected_msg.is_face_detected = is_face_detected_data
                                
                            # IsVisible
                            data_list = self.memProxy.getDataList(is_visible)
                            if (len (data_list)) > 0:
                                is_visible_data = self.memProxy.getData(is_visible)
                                if is_visible_data != None:
                                    people_detected_msg.is_visible = is_visible_data
                        
                            # NotSeenSince
                            data_list = self.memProxy.getDataList(not_seen_since)
                            if (len (data_list)) > 0:
                                not_seen_since_data = self.memProxy.getData(not_seen_since)
                                if not_seen_since_data != None:
                                    people_detected_msg.not_seen_since = not_seen_since_data

                            # PresentSince
                            data_list = self.memProxy.getDataList(present_since)
                            if (len (data_list)) > 0:
                                present_since_data = self.memProxy.getData(present_since)
                                if present_since_data != None:
                                    people_detected_msg.present_since = present_since_data

                            # RealHeight
                            data_list = self.memProxy.getDataList(real_height)
                            if (len (data_list)) > 0:
                                real_height_data = self.memProxy.getData(real_height)
                                if real_height_data != None:
                                    people_detected_msg.real_height = real_height_data
                            
                            # ShirtColor
                            data_list = self.memProxy.getDataList(shirt_color)
                            if (len (data_list)) > 0:
                                shirt_color_data = self.memProxy.getData(shirt_color)
                                if shirt_color_data != None:
                                    people_detected_msg.shirt_color = shirt_color_data

                            # ShirtColorHSV
                            data_list = self.memProxy.getDataList(shirt_color_hsv)
                            if (len (data_list)) > 0:
                                shirt_color_hsv_data = self.memProxy.getData(shirt_color_hsv)
                                if shirt_color_hsv_data != None:
                                    people_detected_msg.hsv.x = shirt_color_hsv_data[0]
                                    people_detected_msg.hsv.y = shirt_color_hsv_data[1]
                                    people_detected_msg.hsv.z = shirt_color_hsv_data[2]
                            
                            self.peoplePerceptionPub.publish(people_perception_msg)
                        self.pre_people_detected_list = people_detected_list[1]

            except RuntimeError, e:
                pass

if __name__ == '__main__':
    peoplePerception = NaoqiPeoplePerception()
    peoplePerception.start()
    rospy.spin()
