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
from std_srvs.srv import (
    EmptyResponse,
    Empty,
    SetBoolResponse,
    SetBool,
    TriggerResponse,
    Trigger
)
from naoqi_bridge_msgs.srv import (
    SetFloatResponse,
    SetFloat,
    GetFloatResponse,
    GetFloat
    )

class NaoqiPeoplePerception (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_people_perception')
        self.connectNaoQi()
        
        self.getMaximumDetectionRangeSrv = rospy.Service("get_maximum_detection_range", GetFloat, self.handleGetMaximumDetectionRange)
        self.getTimeBeforePersonDisappearsSrv = rospy.Service("get_time_before_person_disappears", GetFloat, self.handleGetTimeBeforePersonDisappears)
        self.getTimeBeforeVisiblePersonDisappearsSrv = rospy.Service("get_time_before_visible_person_disappears", GetFloat, self.handleGetTimeBeforeVisiblePersonDisappears)
        self.isFaceDetectionEnabledSrv = rospy.Service("is_face_detection_enabled", Trigger, self.handleIsFaceDetectionEnabled)
        self.isFastModeEnabledSrv = rospy.Service("get_fast_mode_enabled", Trigger, self.handleIsFastModeEnabled)
        self.isGraphicalDisplayEnabledSrv = rospy.Service("get_graphical_display_enabled", Trigger, self.handleIsGraphicalDisplayEnabled)
        self.isMovementDetectionEnabledSrv = rospy.Service("get_movement_detection_enabled", Trigger, self.handleIsMovementDetectionEnabled)
        self.resetPopulationSrv = rospy.Service("reset_population", Empty, self.handleResetPopulation)
        self.setFastModeEnabledSrv = rospy.Service("set_fast_mode_enabled", SetBool, self.handleSetFastModeEnabled)
        self.setGraphicalDisplayEnabledSrv = rospy.Service("set_graphical_display_enabled", SetBool, self.handleSetGraphicalDisplayEnabled)
        self.setMaximumDetectionRangeSrv = rospy.Service("set_maximum_detection_range_enabled", SetFloat, self.handleSetMaximumDetectionRange)
        self.setMovementDetectionEnabledSrv = rospy.Service("set_movement_detection_enabled", SetBool, self.handleSetMovementDetectionEnabled)
        self.setTimeBeforePersonDisappearsSrv = rospy.Service("set_time_before_person_disappears", SetFloat, self.handleSetTimeBeforePersonDisappears)
        self.setTimeBeforeVisiblePersonDisappearsSrv = rospy.Service("set_time_before_visible_person_disappears", SetFloat, self.handleSetTimeBeforeVisiblePersonDisappears)
        rospy.loginfo("naoqi_peoplePerception is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.peoplePerceptionProxy = self.get_proxy("ALPeoplePerception")
        if self.peoplePerceptionProxy is None:
            rospy.logerr("Could not get a proxy to ALPeoplePerception")
            exit(1)
            
    def handleGetMaximumDetectionRange(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.peoplePerceptionProxy.getMaximumDetectionRange()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTimeBeforePersonDisappears(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.peoplePerceptionProxy.getTimeBeforePersonDisappears()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTimeBeforeVisiblePersonDisappears(self, req):
        try:
            res = GetFloatResponse()
            res.data = self.peoplePerceptionProxy.getTimeBeforeVisiblePersonDisappears()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleIsFaceDetectionEnabled(self, req):
        try:
            res = TriggerResponse()
            res.success = self.peoplePerceptionProxy.isFaceDetectionEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsFastModeEnabled(self, req):
        try:
            res = TriggerResponse()
            res.success = self.peoplePerceptionProxy.isFastModeEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsGraphicalDisplayEnabled(self, req):
        try:
            res = TriggerResponse()
            res.success = self.peoplePerceptionProxy.isGraphicalDisplayEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsMovementDetectionEnabled(self, req):
        try:
            res = TriggerResponse()
            res.success = self.peoplePerceptionProxy.isMovementDetectionEnabled()
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
        res = SetBoolResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setFastModeEnabled(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetGraphicalDisplayEnabled(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setGraphicalDisplayEnabled(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMaximumDetectionRange(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setMaximumDetectionRange(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMovementDetectionEnabled(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setMovementDetectionEnabled(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTimeBeforePersonDisappears(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setTimeBeforePersonDisappears(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTimeBeforeVisiblePersonDisappears(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.peoplePerceptionProxy.setTimeBeforeVisiblePersonDisappears(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

if __name__ == '__main__':
    peoplePerception = NaoqiPeoplePerception()
    rospy.spin()
