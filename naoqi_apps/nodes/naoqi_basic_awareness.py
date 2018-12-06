#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    SetBoolResponse, 
    SetBool,
    TriggerResponse,
    Trigger,
    EmptyResponse,
    Empty
)
from naoqi_bridge_msgs.srv import (
    GetStringResponse,
    GetString,
    SetStringResponse,
    SetString,
    SetFloatResponse,
    SetFloat,
    GetFloatResponse,
    GetFloat,
    SetStimulusDetectionEnabledResponse,
    SetStimulusDetectionEnabled,
    TriggerStimulusResponse,
    TriggerStimulus
)

class NaoqiBasicAwareness(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_basic_awareness')
        self.connectNaoQi()
        
        self.SetEnabledSrv = rospy.Service("basic_awareness/set_enabled", SetBool, self.handleSetEnabledSrv)
        self.IsEnabledSrv = rospy.Service("basic_awareness/is_enabled", Trigger, self.handleIsEnabledSrv)
        self.IsRunningSrv = rospy.Service("basic_awareness/is_running", Trigger, self.handleIsRunningSrv)
        self.PauseAwarenessSrv = rospy.Service("basic_awareness/pause_awareness", Empty, self.handlePauseAwarenessSrv)
        self.ResumeAwarenessSrv = rospy.Service("basic_awareness/resume_awareness", Empty, self.handleResumeAwarenessSrv)
        self.IsAwarenessPausedSrv = rospy.Service("basic_awareness/is_awareness_paused", Trigger, self.handleIsAwarenessPausedSrv)
        self.IsStimulusDetectionEnabledSrv = rospy.Service("basic_awareness/is_stimulus_detection_enabled", SetString, self.handleIsStimulusDetectionEnabledSrv)
        self.SetStimulusDetectionEnabledSrv = rospy.Service("basic_awareness/set_stimulus_detection_enabled", SetStimulusDetectionEnabled, self.handleSetStimulusDetectionEnabledSrv)
        self.SetEngagementModeSrv = rospy.Service("basic_awareness/set_engagement_mode", SetString, self.handleSetEngagementModeSrv)
        self.GetEngagementModeSrv = rospy.Service("basic_awareness/get_engagement_mode", GetString, self.handleGetEngagementModeSrv)
        self.EngagePersonSrv = rospy.Service("basic_awareness/engage_person", SetFloat, self.handleEngagePersonSrv)
        self.TriggerStimulusSrv = rospy.Service("basic_awareness/trigger_stimulus", TriggerStimulus, self.handleTriggerStimulusSrv)
        self.SetTrackingModeSrv = rospy.Service("basic_awareness/set_tracking_mode", SetString, self.handleSetTrackingModeSrv)
        self.GetTrackingModeSrv = rospy.Service("basic_awareness/get_tracking_mode", GetString, self.handleGetTrackingModeSrv)
        self.SetLookStimulusSpeedSrv = rospy.Service("basic_awareness/set_look_stimulus_speed", SetFloat, self.handleSetLookStimulusSpeedSrv)
        self.GetLookStimulusSpeedSrv = rospy.Service("basic_awareness/get_look_stimulus_speed", GetFloat, self.handleGetLookStimulusSpeedSrv)
        self.SetLookBackSpeedSrv = rospy.Service("basic_awareness/set_look_back_speed", SetFloat, self.handleSetLookBackSpeedSrv)
        self.GetLookBackSpeedSrv = rospy.Service("basic_awareness/get_look_back_speed", GetFloat, self.handleGetLookBackSpeedSrv)
        self.ResetAllParametersSrv = rospy.Service("basic_awareness/reset_all_parameters", Empty, self.handleResetAllParametersSrv)
        
        rospy.loginfo("naoqi_basic_awareness initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.basicAwarenessProxy = self.get_proxy("ALBasicAwareness")
        if self.basicAwarenessProxy is None:
            exit(1)
    
    def handleSetEnabledSrv(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.basicAwarenessProxy.setEnabled(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleIsEnabledSrv(self, req):
        try:
            res = TriggerResponse()
            res.success = self.basicAwarenessProxy.isEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsRunningSrv(self, req):
        res = TriggerResponse()
        res.success = False
        try:
            res.success = self.basicAwarenessProxy.isEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handlePauseAwarenessSrv(self, req):
        try:
            res = EmptyResponse()
            self.basicAwarenessProxy.pauseAwareness()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleResumeAwarenessSrv(self, req):
        try:
            res = EmptyResponse()
            self.basicAwarenessProxy.resumeAwareness()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsAwarenessPausedSrv(self, req):
        res = TriggerResponse()
        res.success = False
        try:
            res.success = self.basicAwarenessProxy.isAwarenessPaused()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleIsStimulusDetectionEnabledSrv(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            res.success = self.basicAwarenessProxy.isStimulusDetectionEnabled(req.data)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetStimulusDetectionEnabledSrv(self, req):
        res = SetStimulusDetectionEnabledResponse()
        res.success = False
        try:
            self.basicAwarenessProxy.setStimulusDetectionEnabled(req.data, req.status)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetEngagementModeSrv(self, req):
        res = SetStringResponse()
        res.success = False
        try: 
            self.basicAwarenessProxy.setEngagementMode(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleGetEngagementModeSrv(self, req):
        res = GetStringResponse()
        try: 
            res.data = self.basicAwarenessProxy.getEngagementMode()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleEngagePersonSrv(self, req):
        res = SetFloatResponse()
        try: 
            res.success = self.basicAwarenessProxy.engagePerson(int(req.data))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleTriggerStimulusSrv(self, req):
        res = TriggerStimulusResponse()
        try:
            res.people_id = self.basicAwarenessProxy.triggerStimulus([req.position.x, req.position.y, req.position.z])
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetTrackingModeSrv(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.basicAwarenessProxy.setTrackingMode(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleGetTrackingModeSrv(self, req):
        res = GetStringResponse()
        try:
            res.data = self.basicAwarenessProxy.getTrackingMode()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetLookStimulusSpeedSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.basicAwarenessProxy.setParameter("LookStimulusSpeed", req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetLookStimulusSpeedSrv(self, req):
        res = GetFloatResponse()
        try:
            res.data = self.basicAwarenessProxy.getParameter("LookStimulusSpeed")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetLookBackSpeedSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.basicAwarenessProxy.setParameter("LookBackSpeed", req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetLookBackSpeedSrv(self, req):
        res = GetFloatResponse()
        try:
            res.data = self.basicAwarenessProxy.getParameter("LookBackSpeed")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleResetAllParametersSrv(self, req):
        try:
            res = EmptyResponse()
            self.basicAwarenessProxy.resetAllParameters()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

if __name__ == '__main__':
    basic_awareness = NaoqiBasicAwareness()
    rospy.spin()
