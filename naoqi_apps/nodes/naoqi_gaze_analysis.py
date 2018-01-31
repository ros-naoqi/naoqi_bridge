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
    SetFloatResponse,
    SetFloat,
    GetFloatResponse,
    GetFloat)

class NaoqiGazeAnalysis (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_gaze_analysis')
        self.connectNaoQi()
        self.setToleranceSrv = rospy.Service("set_gaze_analysis_tolerance", SetFloat, self.handleSetGazeAnalysisTolerance)
        self.getToleranceSrv = rospy.Service("get_gaze_analysis_tolerance", GetFloat, self.handleGetGazeAnalysisTolerance)
        rospy.loginfo("naoqi_gaze_analysis is initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.gazeProxy = self.get_proxy("ALGazeAnalysis")
        if self.gazeProxy is None:
            rospy.logerr("Could not get a proxy to ALGazeAnalysis")
            exit(1)

    def handleSetGazeAnalysisTolerance(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.gazeProxy.setTolerance(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleGetGazeAnalysisTolerance(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.gazeProxy.getTolerance()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

if __name__ == '__main__':
    gaze_analysis = NaoqiGazeAnalysis()
    rospy.spin()
