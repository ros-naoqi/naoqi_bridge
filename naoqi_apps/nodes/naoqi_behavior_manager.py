#!/usr/bin/env python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.srv import (
    SetStringResponse, 
    SetString
)

class NaoqiBehaviorManager(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_behavior_manager')
        self.connectNaoQi()
        
        self.StopBehaviorSrv = rospy.Service("behavior_manager/stop_behavior", SetString, self.handleStopBehaviorSrv)
        self.IsBehaviorRunningSrv = rospy.Service("behavior_manager/is_behavior_running", SetString, self.handleIsBehaviorRunningSrv)
        rospy.loginfo("naoqi_behavior_manager initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.behaviorManagerProxy = self.get_proxy("ALBehaviorManager")
        if self.behaviorManagerProxy is None:
            rospy.logerr("Could not get a proxy to ALBehaviorManager")
            exit(1)
    
    def handleStopBehaviorSrv(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.behaviorManagerProxy.stopBehavior(req.data)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            rospy.signal_shutdown("service call failed")

    def handleIsBehaviorRunningSrv(self, req):
        try:
            res = SetStringResponse()
            res.success = self.behaviorManagerProxy.isBehaviorRunning(req.data)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            rospy.signal_shutdown("service call failed")

if __name__ == '__main__':
    behavior_manager = NaoqiBehaviorManager()
    rospy.spin()
