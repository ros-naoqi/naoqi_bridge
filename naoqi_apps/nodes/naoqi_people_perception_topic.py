#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.msg import PeoplePerception

class NaoqiPeoplePerceptionTopic (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_people_perception_topic')
        self.connectNaoQi()
        self.rate = rospy.Rate(10) 
        self.pre_people_detected_list = None
        self.peoplePerceptionPub = rospy.Publisher("people_perception", PeoplePerception, queue_size=10)
        rospy.loginfo("naoqi_people_perception_topic is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        if self.memProxy is None:
            rospy.logerr("Could not get a proxy to ALMemory")
            exit(1)
            
    def run(self):
        while self.is_looping():
            try:
                if self.peoplePerceptionPub.get_num_connections() > 0:
                    # ALPeoplePerception
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

                            # ALFaceCharacteristics
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
                                    face_characteristics_msg.person_facial_parts.left_eye_center_point.x = facial_parts_data[0][0][0]
                                    face_characteristics_msg.person_facial_parts.left_eye_center_point.y = facial_parts_data[0][0][1]
                                    face_characteristics_msg.person_facial_parts.left_eye_center_confidence = facial_parts_data[0][1]

                                    face_characteristics_msg.person_facial_parts.right_eye_center_point.x = facial_parts_data[1][0][0]
                                    face_characteristics_msg.person_facial_parts.right_eye_center_point.y = facial_parts_data[1][0][1]
                                    face_characteristics_msg.person_facial_parts.right_eye_center_confidence = facial_parts_data[1][1]

                                    face_characteristics_msg.person_facial_parts.mouth_center_point.x = facial_parts_data[2][0][0]
                                    face_characteristics_msg.person_facial_parts.mouth_center_point.y = facial_parts_data[2][0][1]
                                    face_characteristics_msg.person_facial_parts.mouth_center_confidence = facial_parts_data[2][1]

                                    face_characteristics_msg.person_facial_parts.left_eye_inner_limit.x = facial_parts_data[3][0][0]
                                    face_characteristics_msg.person_facial_parts.left_eye_inner_limit.y = facial_parts_data[3][0][1]
                                    face_characteristics_msg.person_facial_parts.left_eye_inner_limit_confidence = facial_parts_data[3][1]

                                    face_characteristics_msg.person_facial_parts.left_eye_outer_limit.x = facial_parts_data[4][0][0]
                                    face_characteristics_msg.person_facial_parts.left_eye_outer_limit.y = facial_parts_data[4][0][1]
                                    face_characteristics_msg.person_facial_parts.left_eye_outer_limit_confidence = facial_parts_data[4][1]

                                    face_characteristics_msg.person_facial_parts.right_eye_inner_limit.x = facial_parts_data[5][0][0]
                                    face_characteristics_msg.person_facial_parts.right_eye_inner_limit.y = facial_parts_data[5][0][1]
                                    face_characteristics_msg.person_facial_parts.right_eye_inner_limit_confidence = facial_parts_data[5][1]

                                    face_characteristics_msg.person_facial_parts.right_eye_outer_limit.x = facial_parts_data[6][0][0]
                                    face_characteristics_msg.person_facial_parts.right_eye_outer_limit.y = facial_parts_data[6][0][1]
                                    face_characteristics_msg.person_facial_parts.right_eye_outer_limit_confidence = facial_parts_data[6][1]

                                    face_characteristics_msg.person_facial_parts.mouth_left_limit.x = facial_parts_data[7][0][0]
                                    face_characteristics_msg.person_facial_parts.mouth_left_limit.y = facial_parts_data[7][0][1]
                                    face_characteristics_msg.person_facial_parts.mouth_left_limit_confidence = facial_parts_data[7][1]

                                    face_characteristics_msg.person_facial_parts.mouth_right_limit.x = facial_parts_data[8][0][0]
                                    face_characteristics_msg.person_facial_parts.mouth_right_limit.y = facial_parts_data[8][0][1]
                                    face_characteristics_msg.person_facial_parts.mouth_right_limit_confidence = facial_parts_data[8][1]

                                    face_characteristics_msg.person_facial_parts.nose_left_limit.x = facial_parts_data[9][0][0]
                                    face_characteristics_msg.person_facial_parts.nose_left_limit.y = facial_parts_data[9][0][1]
                                    face_characteristics_msg.person_facial_parts.nose_left_limit_confidence = facial_parts_data[9][1]

                                    face_characteristics_msg.person_facial_parts.nose_right_limit.x = facial_parts_data[10][0][0]
                                    face_characteristics_msg.person_facial_parts.nose_right_limit.y = facial_parts_data[10][0][1]
                                    face_characteristics_msg.person_facial_parts.nose_right_limit_confidence = facial_parts_data[10][1]

                                    face_characteristics_msg.person_facial_parts.mouth_upper_limit.x = facial_parts_data[11][0][0]
                                    face_characteristics_msg.person_facial_parts.mouth_upper_limit.y = facial_parts_data[11][0][1]
                                    face_characteristics_msg.person_facial_parts.mouth_upper_limit_confidence = facial_parts_data[11][1]

                            # GenderProperties
                            data_list =  self.memProxy.getDataList(gender)
                            if (len (data_list)) > 0:
                                gender_data = self.memProxy.getData(gender)
                                if (len (gender_data) > 0):
                                    face_characteristics_msg.person_gender.gender = int(gender_data[0])
                                    face_characteristics_msg.person_gender.confidence = gender_data[1]

                            # SmileProperties
                            data_list =  self.memProxy.getDataList(smile)
                            if (len (data_list)) > 0:
                                smile_data = self.memProxy.getData(smile)
                                if (len (smile_data) > 0):
                                    face_characteristics_msg.person_smile.smile_degree = smile_data[0]
                                    face_characteristics_msg.person_smile.confidence = smile_data[1]
                            
                            # ALGazeAnalysis
                            gaze_analysis_msg = people_perception_msg.gaze_analysis

                            eye_opening_degree = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/EyeOpeningDegree"
                            gaze_direction = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/GazeDirection"
                            head_angles = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/HeadAngles"
                            is_looking_at_robot = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/IsLookingAtRobot"
                            looking_at_robot_score = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/LookingAtRobotScore"

                            # EyeOpeningDegree
                            data_list = self.memProxy.getDataList(eye_opening_degree)
                            if (len (data_list)) > 0:
                                eye_opening_degree_data = self.memProxy.getData(eye_opening_degree)
                                if len(eye_opening_degree_data) > 0:
                                    gaze_analysis_msg.eye_opening_degree.append(eye_opening_degree_data[0])
                                    gaze_analysis_msg.eye_opening_degree.append(eye_opening_degree_data[1])

                            # GazeDirection
                            data_list = self.memProxy.getDataList(gaze_direction)
                            if (len (data_list)) > 0:
                                gaze_direction_data = self.memProxy.getData(gaze_direction)
                                if (len (gaze_direction_data) > 0):
                                    gaze_analysis_msg.gaze_direction.append(gaze_direction_data[0])
                                    gaze_analysis_msg.gaze_direction.append(gaze_direction_data[1])

                            # HeadAngles
                            data_list = self.memProxy.getDataList(head_angles)
                            if (len (data_list)) > 0:
                                head_angles_data = self.memProxy.getData(head_angles)
                                if (len (head_angles_data) > 0):
                                    gaze_analysis_msg.head_angles.append(head_angles_data[0])
                                    gaze_analysis_msg.head_angles.append(head_angles_data[1])
                                    gaze_analysis_msg.head_angles.append(head_angles_data[2])

                            # IsLookingAtRobot
                            data_list = self.memProxy.getDataList(is_looking_at_robot)
                            if (len (data_list)) > 0:
                                is_looking_at_robot_data = self.memProxy.getData(is_looking_at_robot)
                                if is_looking_at_robot_data is not None:
                                    gaze_analysis_msg.is_looking_at_robot = is_looking_at_robot_data

                            # LookingAtRobotScore
                            data_list = self.memProxy.getDataList(looking_at_robot_score)
                            if (len (data_list)) > 0:
                                looking_at_robot_score_data = self.memProxy.getData(looking_at_robot_score)
                                if looking_at_robot_score_data is not None:
                                    gaze_analysis_msg.looking_at_robot_score = looking_at_robot_score_data

                            # ALEngagementZones
                            engagement_zones = "PeoplePerception/Person/" + str(people_detected_msg.people_id) + "/EngagementZone"
                            data_list = self.memProxy.getDataList(engagement_zones)
                            if (len (data_list)) > 0:
                                engagement_zones_data = self.memProxy.getData(engagement_zones)
                                if engagement_zones_data is not None:
                                    people_perception_msg.engagement_zone_of_person = engagement_zones_data

                            self.peoplePerceptionPub.publish(people_perception_msg)
                        self.pre_people_detected_list = people_detected_list[1]

            except RuntimeError, e:
                pass
            self.rate.sleep()

if __name__ == '__main__':
    peoplePerception = NaoqiPeoplePerceptionTopic()
    peoplePerception.start()
    rospy.spin()
