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
from naoqi_bridge_msgs.msg import FaceDetected
from naoqi_bridge_msgs.srv import (
ClearDatabaseResponse,
ClearDatabase,
ForgetPersonResponse,
ForgetPerson,
GetLearnedFacesListResponse,
GetLearnedFacesList,
GetRecognitionConfidenceThresholdResponse,
GetRecognitionConfidenceThreshold,
IsRecognitionEnabledResponse,
IsRecognitionEnabled,
IsTrackingEnabledResponse,
IsTrackingEnabled,
LearnFaceResponse,
LearnFace,
ReLearnFaceResponse,
ReLearnFace,
SetRecognitionConfidenceThresholdResponse,
SetRecognitionConfidenceThreshold,
SetRecognitionEnabledResponse,
SetRecognitionEnabled,
SetTrackingEnabledResponse,
SetTrackingEnabled)

class NaoqiFaceDetection (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_face_detection')
        self.connectNaoQi()
        self.faceDetectedPub = rospy.Publisher("face_detected", FaceDetected, queue_size=10)
        self.clearDatabaseSrv = rospy.Service("clear_database", ClearDatabase, self.handleClearDatabase)
        self.forgetPersonSrv = rospy.Service("forget_person", ForgetPerson, self.handleForgetPerson)
        self.getLearnedFacesListSrv = rospy.Service("get_learned_facesList", GetLearnedFacesList, self.handleGetLearnedFacesList)
        self.getRecognitionConfidenceThresholdSrv = rospy.Service("get_recognition_confidence_threshold", GetRecognitionConfidenceThreshold, self.handleGetRecognitionConfidenceThreshold)
        self.isRecognitionEnabledSrv = rospy.Service("is_recognition_enabled", IsRecognitionEnabled, self.handleIsRecognitionEnabled)
        self.isTrackingEnabledSrv = rospy.Service("is_tracking_enabled", IsTrackingEnabled, self.handleIsTrackingEnabled)
        self.learnFaceSrv = rospy.Service("learn_face", LearnFace, self.handleLearnFace)
        self.reLearnFaceSrv = rospy.Service("relearn_face", ReLearnFace, self.handleReLearnFace)
        self.setRecognitionConfidenceThresholdSrv = rospy.Service("set_recognition_confidence_threshold", SetRecognitionConfidenceThreshold, self.handleSetRecognitionConfidenceThreshold)
        self.setRecognitionEnabledSrv = rospy.Service("set_recognition_enabled", SetRecognitionEnabled, self.handleSetRecognitionEnabled)
        self.setTrackingEnabledSrv = rospy.Service("set_tracking_enabled", SetTrackingEnabled, self.handleSetTrackingEnabled)
        rospy.loginfo("naoqi_face_detection is initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.faceProxy = self.get_proxy("ALFaceDetection")
        if self.memProxy is None or self.faceProxy is None:
            exit(1)

    def handleClearDatabase(self, req):
        res = ClearDatabaseResponse()
        res.success = False
        try:
            self.faceProxy.clearDatabase()
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleForgetPerson(self, req):
        try:
            res = ForgetPersonResponse()
            res.success = self.faceProxy.forgetPerson(req.name)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetLearnedFacesList(self, req):
        try:
            res = GetLearnedFacesListResponse()
            faces_list = self.faceProxy.getLearnedFacesList()
            if faces_list != None and (len(faces_list) > 0):
                for i in range(len(faces_list)):
                    res.names.append(faces_list[i])
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetRecognitionConfidenceThreshold(self, req):
        try:
            res = GetRecognitionConfidenceThresholdResponse()
            res.threshold = self.faceProxy.getRecognitionConfidenceThreshold()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsRecognitionEnabled(self, req):
        try:
            res = IsRecognitionEnabledResponse()
            res.status = self.faceProxy.isRecognitionEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsTrackingEnabled(self, req):
        try:
            res = IsTrackingEnabledResponse()
            res.status = self.faceProxy.isTrackingEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleLearnFace(self, req):
        try:
            res = LearnFaceResponse()
            res.success = self.faceProxy.learnFace(req.name)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleReLearnFace(self, req):
        try:
            res = ReLearnFaceResponse()
            self.faceProxy.reLearnFace(req.name)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
    def handleSetRecognitionConfidenceThreshold(self, req):
        res = SetRecognitionConfidenceThresholdResponse()
        res.success = False
        try:
            self.faceProxy.setRecognitionConfidenceThreshold(req.threshold)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetRecognitionEnabled(self, req):
        res = SetRecognitionEnabledResponse()
        res.success = False
        try:
            self.faceProxy.setRecognitionEnabled(req.enable)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTrackingEnabled(self, req):
        res = SetTrackingEnabledResponse()
        res.success = False
        try:
            self.faceProxy.setTrackingEnabled(req.enable)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                data_list = self.memProxy.getDataList("FaceDetected")
                if data_list != None and len(data_list) > 0:
                    face_detected_data = self.memProxy.getData("FaceDetected")
                    if face_detected_data != None and (len(face_detected_data) > 0):
                        face_detected_msg = FaceDetected()
                        if (len(face_detected_data[1]) - 1) > 0:
                            for i in range (len(face_detected_data[1]) - 1):
                                face_detected_msg.header.stamp = rospy.get_rostime()
                                
                                face_detected_msg.face_info.shape_info.alpha = face_detected_data[1][i][0][1]
                                face_detected_msg.face_info.shape_info.beta = face_detected_data[1][i][0][2]
                                face_detected_msg.face_info.shape_info.size_x = face_detected_data[1][i][0][3] 
                                face_detected_msg.face_info.shape_info.size_y = face_detected_data[1][i][0][4]
                                
                                face_detected_msg.face_info.extra_info.face_id = face_detected_data[1][i][1][0]
                                face_detected_msg.face_info.extra_info.recognition_score = face_detected_data[1][i][1][1]
                                face_detected_msg.face_info.extra_info.face_label = face_detected_data[1][i][1][2]
                                
                                face_detected_msg.face_info.extra_info.left_eye.eye_center_x = face_detected_data[1][i][1][3][0] 
                                face_detected_msg.face_info.extra_info.left_eye.eye_center_y = face_detected_data[1][i][1][3][1]
                                face_detected_msg.face_info.extra_info.left_eye.nose_side_limit_x = face_detected_data[1][i][1][3][2] 
                                face_detected_msg.face_info.extra_info.left_eye.nose_side_limit_y = face_detected_data[1][i][1][3][3]
                                face_detected_msg.face_info.extra_info.left_eye.ear_side_limit_x = face_detected_data[1][i][1][3][4]
                                face_detected_msg.face_info.extra_info.left_eye.ear_side_limit_y = face_detected_data[1][i][1][3][5]

                                face_detected_msg.face_info.extra_info.right_eye.eye_center_x = face_detected_data[1][i][1][4][0] 
                                face_detected_msg.face_info.extra_info.right_eye.eye_center_y = face_detected_data[1][i][1][4][1]
                                face_detected_msg.face_info.extra_info.right_eye.nose_side_limit_x = face_detected_data[1][i][1][4][2] 
                                face_detected_msg.face_info.extra_info.right_eye.nose_side_limit_y = face_detected_data[1][i][1][4][3]
                                face_detected_msg.face_info.extra_info.right_eye.ear_side_limit_x = face_detected_data[1][i][1][4][4]
                                face_detected_msg.face_info.extra_info.right_eye.ear_side_limit_y = face_detected_data[1][i][1][4][5]
                                
                                face_detected_msg.face_info.extra_info.nose.bottom_center_limit_x = face_detected_data[1][i][1][7][0]
                                face_detected_msg.face_info.extra_info.nose.bottom_center_limit_y = face_detected_data[1][i][1][7][1]
                                face_detected_msg.face_info.extra_info.nose.bottom_left_limit_x = face_detected_data[1][i][1][7][2]
                                face_detected_msg.face_info.extra_info.nose.bottom_left_limit_y = face_detected_data[1][i][1][7][3]
                                face_detected_msg.face_info.extra_info.nose.bottom_right_limit_x = face_detected_data[1][i][1][7][4]
                                face_detected_msg.face_info.extra_info.nose.bottom_right_limit_y = face_detected_data[1][i][1][7][5]

                                face_detected_msg.face_info.extra_info.mouth.left_limit_x = face_detected_data[1][i][1][8][0]
                                face_detected_msg.face_info.extra_info.mouth.left_limit_y = face_detected_data[1][i][1][8][1]
                                face_detected_msg.face_info.extra_info.mouth.right_limit_x = face_detected_data[1][i][1][8][2]
                                face_detected_msg.face_info.extra_info.mouth.right_limit_y = face_detected_data[1][i][1][8][3]
                                face_detected_msg.face_info.extra_info.mouth.top_limit_x = face_detected_data[1][i][1][8][4]
                                face_detected_msg.face_info.extra_info.mouth.top_limit_y = face_detected_data[1][i][1][8][5]

                                if len(face_detected_data[1][len(face_detected_data[1]) - 1]) > 0:
                                    face_detected_msg.time_filtered_reco_info.status = face_detected_data[1][len(face_detected_data[1]) - 1][0]
                                    if len(face_detected_data[1][len(face_detected_data[1]) - 1]) > 1:
                                        for i in range(len(face_detected_data[1][len(face_detected_data[1]) - 1][1])):
                                            face_detected_msg.time_filtered_reco_info.face_label.append(face_detected_data[1][len(face_detected_data[1]) - 1][1][i]) 
                            
                                face_detected_msg.camera_pose_in_torso_frame.linear.x = face_detected_data[2][0]
                                face_detected_msg.camera_pose_in_torso_frame.linear.y = face_detected_data[2][1]
                                face_detected_msg.camera_pose_in_torso_frame.linear.z = face_detected_data[2][2]
                                face_detected_msg.camera_pose_in_torso_frame.angular.x = face_detected_data[2][3]
                                face_detected_msg.camera_pose_in_torso_frame.angular.y = face_detected_data[2][4]
                                face_detected_msg.camera_pose_in_torso_frame.angular.z = face_detected_data[2][5]
                                face_detected_msg.camera_pose_in_robot_frame.linear.x = face_detected_data[3][0]
                                face_detected_msg.camera_pose_in_robot_frame.linear.y  = face_detected_data[3][1]
                                face_detected_msg.camera_pose_in_robot_frame.linear.z = face_detected_data[3][2]
                                face_detected_msg.camera_pose_in_robot_frame.angular.x = face_detected_data[3][3]
                                face_detected_msg.camera_pose_in_robot_frame.angular.y = face_detected_data[3][4]
                                face_detected_msg.camera_pose_in_robot_frame.angular.z = face_detected_data[3][5]
                                face_detected_msg.camera_id = face_detected_data[4] 
                                self.faceDetectedPub.publish(face_detected_msg)

            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    face_detection = NaoqiFaceDetection()
    face_detection.start()
    rospy.spin()
