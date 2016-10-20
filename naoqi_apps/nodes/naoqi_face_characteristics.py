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
from std_msgs.msg import Int64
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.srv import (
    AnalyzeFaceCharacteristicsResponse,
    AnalyzeFaceCharacteristics,
    GetSmilingThresholdResponse,
    GetSmilingThreshold,
    SetSmilingThresholdResponse,
    SetSmilingThreshold)
from naoqi_bridge_msgs.msg import FaceCharacteristics

class NaoqiFaceCharacteristics (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_faceCharacteristics')
        self.connectNaoQi()
        self.analyzeFaceCharacteristicsSrv = rospy.Service("analyze_face_characteristics", AnalyzeFaceCharacteristics, self.handleAnalyzeFaceCharacteristics)
        self.getSmilingThresholdSrv = rospy.Service("get_smiling_threshold", GetSmilingThreshold, self.handleGetSmilingThreshold)
        self.setSmilingThresholdSrv = rospy.Service("set_smiling_threshold", SetSmilingThreshold, self.handleSetSmilingThreshold)
        self.faceCharacteristicsPub = rospy.Publisher("face_characteristics", FaceCharacteristics, queue_size=10)        
        self.personSmilingPub = rospy.Publisher("person_smiling", Int64, queue_size=10)

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.faceC = self.get_proxy("ALFaceCharacteristics")
        if self.memProxy is None and self.faceC is None:
            exit(1)

    def handleAnalyzeFaceCharacteristics (self, req):
        res = AnalyzeFaceCharacteristicsResponse()
        res.success = False
        try:
            res.success = self.faceC.analyzeFaceCharacteristics(req.people_id)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleGetSmilingThreshold (self, req):
        try:
            res = GetSmilingThresholdResponse()
            res.threshold = self.faceC.getSmilingThreshold() 
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetSmilingThreshold (self, req):
        res = SetSmilingThresholdResponse()
        res.success = False
        try:
            self.faceC.setSmilingThreshold(req.threshold)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                age_data = None
                expression_data = None
                facial_parts_data = None
                gender_data = None 
                smile_data = None 

                People_ID = self.memProxy.getData("PeoplePerception/VisiblePeopleList")
                if (len(People_ID)) > 0:
                    for i in range(len(People_ID)):
                        face_characteristics_msg = FaceCharacteristics()
                        age = "PeoplePerception/Person/" + str(People_ID[i]) + "/AgeProperties"
                        expression = "PeoplePerception/Person/" + str(People_ID[i]) + "/ExpressionProperties"
                        facial_parts = "PeoplePerception/Person/" + str(People_ID[i]) + "/FacialPartsProperties"
                        gender = "PeoplePerception/Person/" + str(People_ID[i]) + "/GenderProperties"
                        smile = "PeoplePerception/Person/" + str(People_ID[i]) + "/SmileProperties"
                        
                        face_characteristics_msg.header.stamp = rospy.get_rostime()
                        face_characteristics_msg.people_id = People_ID[i]
                                    
                        data_list =  self.memProxy.getDataList(age)
                        if (len (data_list)) > 0:
                            age_data = self.memProxy.getData(age)
                            if (len (age_data) > 0):
                                face_characteristics_msg.person_age.age = age_data[0]
                                face_characteristics_msg.person_age.confidence = age_data[1]
                                
                        data_list =  self.memProxy.getDataList(expression)
                        if (len (data_list)) > 0:
                            expression_data = self.memProxy.getData(expression)
                            if (len (expression_data) > 0):
                                face_characteristics_msg.person_expression.neutral = expression_data[0]
                                face_characteristics_msg.person_expression.happy = expression_data[1]
                                face_characteristics_msg.person_expression.surprised = expression_data[2]
                                face_characteristics_msg.person_expression.angry = expression_data[3]
                                face_characteristics_msg.person_expression.sad = expression_data[4]
                              
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
                                            
                        data_list =  self.memProxy.getDataList(gender)
                        if (len (data_list)) > 0:
                            gender_data = self.memProxy.getData(gender)
                            if (len (gender_data) > 0):
                                face_characteristics_msg.person_gender.gender = gender_data[0]
                                face_characteristics_msg.person_gender.confidence = gender_data[1]
                                            
                        data_list =  self.memProxy.getDataList(smile)
                        if (len (data_list)) > 0:
                            smile_data = self.memProxy.getData(smile)
                            if (len (smile_data) > 0):
                                face_characteristics_msg.person_smile.smile_degree = smile_data[0]
                                face_characteristics_msg.person_smile.confidence = smile_data[1]
                        if age_data != None and expression_data != None and facial_parts_data != None and gender_data != None and smile_data != None and (len (age_data) > 0) and (len (expression_data) > 0) and (len (facial_parts_data) > 0) and (len (gender_data) > 0) and (len (smile_data) > 0):
                            self.faceCharacteristicsPub.publish(face_characteristics_msg)

                # FaceCharacteristics/PersonSmiling Event
                data_list = self.memProxy.getDataList("FaceCharacteristics/PersonSmiling")
                if (len (data_list)) > 0:
                    person_smiling_msg = Int64()
                    person_smiling_msg.data = self.memProxy.getData("FaceCharacteristics/PersonSmiling")
                    self.personSmilingPub.publish(person_smiling_msg)

            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    face_characteristics = NaoqiFaceCharacteristics()
    face_characteristics.start()
    rospy.spin()
