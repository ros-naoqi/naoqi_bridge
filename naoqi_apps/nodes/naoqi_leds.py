#!/usr/bin/env python

#                                                                             
#  Copyright 2017 Aldebaran                                                   
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
    FadeRGBWithColorNameResponse, 
    FadeRGBWithColorName,
    ResetLEDResponse,
    ResetLED
)

class NaoqiLeds(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_leds')
        self.connectNaoQi()

        self.FadeRGBWithColorNameSrv = rospy.Service("fade_led_rgb_with_color_name", FadeRGBWithColorName, self.handleFadeRGBWithColorName)        
        self.resetSrv = rospy.Service("reset_led", ResetLED, self.handleResetLed)
        rospy.loginfo("naoqi_leds initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.ledsProxy = self.get_proxy("ALLeds")
        if self.ledsProxy is None:
            exit(1)
    
    def handleFadeRGBWithColorName(self, req):
        res = FadeRGBWithColorNameResponse()
        res.success = False
        color_name = {0: "white", 1: "red", 2: "green", 3: "blue", 4: "yellow", 5: "magenta", 6: "cyan"}
        try:
            self.ledsProxy.fadeRGB(req.name, color_name[req.color_name.name], req.duration_to_fade)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleResetLed(self, req):
        res = ResetLEDResponse()
        res.success = False
        try:
            self.ledsProxy.reset(req.name)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                pass
            except RuntimeError, e:
                print "Error accessing ALLeds, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    leds = NaoqiLeds()
    leds.start()
    rospy.spin()
