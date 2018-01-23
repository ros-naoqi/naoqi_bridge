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
        self.peopleDetectedPub = rospy.Publisher("people_perception/people_detected", PeoplePerception, queue_size=10)
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
                if self.peopleDetectedPub.get_num_connections() > 0:
                    people_detected_list = self.memProxy.getData("PeoplePerception/PeopleDetected")
                    if people_detected_list != None and (len (people_detected_list) > 0) and people_detected_list[1] != self.pre_people_detected_list:
                        people_perception_msg = PeoplePerception()
                        people_perception_msg.header.stamp = rospy.get_rostime()
                        for i in range (len (people_detected_list[1])):
                            people_detected_msg = people_perception_msg.people_detected
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
                            
                            # FaceCharacteristics
                            face_characteristics_msg = people_perception_msg.face_characteristics
                            age = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/AgeProperties"
                            expression = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/ExpressionProperties"
                            facial_parts = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/FacialPartsProperties"
                            gender = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/GenderProperties"
                            smile = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/SmileProperties"

                            # AgeProperties
                            data_list = self.memProxy.getDataList(age)
                            if (len (data_list)) > 0:
                                age_data = self.memProxy.getData(age)
                                if (len (age_data) > 0):
                                    face_characteristics_msg.person_age.age = age_data[0]
                                    face_characteristics_msg.person_age.confidence = age_data[1]

                            # ExpressionProperties
                            data_list =  self.memProxy.getDataList(expression)
                            if (len (data_list)) > 0:
                                expression_data = self.memProxy.getData(expression)
                                if (len (expression_data) > 0):
                                    face_characteristics_msg.person_expression.neutral = expression_data[0]
                                    face_characteristics_msg.person_expression.happy = expression_data[1]
                                    face_characteristics_msg.person_expression.surprised = expression_data[2]
                                    face_characteristics_msg.person_expression.angry = expression_data[3]
                                    face_characteristics_msg.person_expression.sad = expression_data[4]

                            # FacialPartsProperties
                            data_list = self.memProxy.getDataList(facial_parts)
                            if (len (data_list)) > 0:
                                facial_parts_data = self.memProxy.getData(facial_parts)
                                if (len (facial_parts_data) > 0):
                                    face_characteristics_msg.person_facial_parts.left_eye_center_point_x = facial_parts_data[0][0][0]
                                    face_characteristics_msg.person_facial_parts.left_eye_center_point_y = facial_parts_data[0][0][1]
                                    face_characteristics_msg.person_facial_parts.left_eye_center_confidence = facial_parts_data[0][1]

                                    face_characteristics_msg.person_facial_parts.right_eye_center_point_x = facial_parts_data[1][0][0]
                                    face_characteristics_msg.person_facial_parts.right_eye_center_point_y = facial_parts_data[1][0][1]
                                    face_characteristics_msg.person_facial_parts.right_eye_center_confidence = facial_parts_data[1][1]

                                    face_characteristics_msg.person_facial_parts.mouth_center_point_x = facial_parts_data[2][0][0]
                                    face_characteristics_msg.person_facial_parts.mouth_center_point_y = facial_parts_data[2][0][1]
                                    face_characteristics_msg.person_facial_parts.mouth_center_confidence = facial_parts_data[2][1]

                                    face_characteristics_msg.person_facial_parts.left_eye_inner_limit_x = facial_parts_data[3][0][0]
                                    face_characteristics_msg.person_facial_parts.left_eye_inner_limit_y = facial_parts_data[3][0][1]
                                    face_characteristics_msg.person_facial_parts.left_eye_inner_limit_confidence = facial_parts_data[3][1]

                                    face_characteristics_msg.person_facial_parts.left_eye_outer_limit_x = facial_parts_data[4][0][0]
                                    face_characteristics_msg.person_facial_parts.left_eye_outer_limit_y = facial_parts_data[4][0][1]
                                    face_characteristics_msg.person_facial_parts.left_eye_outer_limit_confidence = facial_parts_data[4][1]

                                    face_characteristics_msg.person_facial_parts.right_eye_inner_limit_x = facial_parts_data[5][0][0]
                                    face_characteristics_msg.person_facial_parts.right_eye_inner_limit_y = facial_parts_data[5][0][1]
                                    face_characteristics_msg.person_facial_parts.right_eye_inner_limit_confidence = facial_parts_data[5][1]

                                    face_characteristics_msg.person_facial_parts.right_eye_outer_limit_x = facial_parts_data[6][0][0]
                                    face_characteristics_msg.person_facial_parts.right_eye_outer_limit_y = facial_parts_data[6][0][1]
                                    face_characteristics_msg.person_facial_parts.right_eye_outer_limit_confidence = facial_parts_data[6][1]

                                    face_characteristics_msg.person_facial_parts.mouth_left_limit_x = facial_parts_data[7][0][0]
                                    face_characteristics_msg.person_facial_parts.mouth_left_limit_y = facial_parts_data[7][0][1]
                                    face_characteristics_msg.person_facial_parts.mouth_left_limit_confidence = facial_parts_data[7][1]

                                    face_characteristics_msg.person_facial_parts.mouth_right_limit_x = facial_parts_data[8][0][0]
                                    face_characteristics_msg.person_facial_parts.mouth_right_limit_y = facial_parts_data[8][0][1]
                                    face_characteristics_msg.person_facial_parts.mouth_right_limit_confidence = facial_parts_data[8][1]

                                    face_characteristics_msg.person_facial_parts.nose_left_limit_x = facial_parts_data[9][0][0]
                                    face_characteristics_msg.person_facial_parts.nose_left_limit_y = facial_parts_data[9][0][1]
                                    face_characteristics_msg.person_facial_parts.nose_left_limit_confidence = facial_parts_data[9][1]

                                    face_characteristics_msg.person_facial_parts.nose_right_limit_x = facial_parts_data[10][0][0]
                                    face_characteristics_msg.person_facial_parts.nose_right_limit_y = facial_parts_data[10][0][1]
                                    face_characteristics_msg.person_facial_parts.nose_right_limit_confidence = facial_parts_data[10][1]

                                    face_characteristics_msg.person_facial_parts.mouth_upper_limit_x = facial_parts_data[11][0][0]
                                    face_characteristics_msg.person_facial_parts.mouth_upper_limit_y = facial_parts_data[11][0][1]
                                    face_characteristics_msg.person_facial_parts.mouth_upper_limit_confidence = facial_parts_data[11][1]

                            # GenderProperties
                            data_list =  self.memProxy.getDataList(gender)
                            if (len (data_list)) > 0:
                                gender_data = self.memProxy.getData(gender)
                                if (len (gender_data) > 0):
                                    face_characteristics_msg.person_gender.gender = gender_data[0]
                                    face_characteristics_msg.person_gender.confidence = gender_data[1]

                            # SmileProperties
                            data_list =  self.memProxy.getDataList(smile)
                            if (len (data_list)) > 0:
                                smile_data = self.memProxy.getData(smile)
                                if (len (smile_data) > 0):
                                    face_characteristics_msg.person_smile.smile_degree = smile_data[0]
                                    face_characteristics_msg.person_smile.confidence = smile_data[1]

                            # TODO: GazeAnalysis

                            # TODO: EngagementZones

                            self.peopleDetectedPub.publish(people_perception_msg)
                        self.pre_people_detected_list = people_detected_list[1]

            except RuntimeError, e:
                pass

if __name__ == '__main__':
    peoplePerception = NaoqiPeoplePerception()
    peoplePerception.start()
    rospy.spin()
