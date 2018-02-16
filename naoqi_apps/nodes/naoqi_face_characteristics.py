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
from naoqi_bridge_msgs.srv import (
    AnalyzeFaceCharacteristicsResponse,
    AnalyzeFaceCharacteristics,
    GetFloatResponse,
    GetFloat,
    SetFloatResponse,
    SetFloat
)
class NaoqiFaceCharacteristics (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_face_characteristics')
        self.connectNaoQi()
        self.analyzeFaceCharacteristicsSrv = rospy.Service("analyze_face_characteristics", AnalyzeFaceCharacteristics, self.handleAnalyzeFaceCharacteristics)
        self.getSmilingThresholdSrv = rospy.Service("get_smiling_threshold", GetFloat, self.handleGetSmilingThreshold)
        self.setSmilingThresholdSrv = rospy.Service("set_smiling_threshold", SetFloat, self.handleSetSmilingThreshold)

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.faceC = self.get_proxy("ALFaceCharacteristics")
        if self.faceC is None:
            rospy.logerr("Could not get a proxy to ALFaceCharacteristics")
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
            res = GetFloatResponse()
            res.data = self.faceC.getSmilingThreshold()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetSmilingThreshold (self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.faceC.setSmilingThreshold(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

if __name__ == '__main__':
    face_characteristics = NaoqiFaceCharacteristics()
    rospy.spin()
