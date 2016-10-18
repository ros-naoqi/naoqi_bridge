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
from std_srvs.srv import(
    EmptyResponse,
    Empty)
from naoqi_bridge_msgs.msg import SoundLocated
from naoqi_bridge_msgs.srv import(
    SetEnergyComputationResponse,
    SetEnergyComputation,
    SetSensitivityResponse,
    SetSensitivity)

class NaoqiSoundLocalization (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_soundLocalization')
        self.connectNaoQi()
        self.soundLocalization = SoundLocated()
        self.pre_sound_localization = None
        self.soundLocalizationPub = rospy.Publisher("soundLocated", SoundLocated, queue_size=10)
        self.subscribeSoundLocalizationSrv = rospy.Service("start_sound_localization", Empty, self.handleSubscribeSoundLocalization)
        self.unsubscribeSoundLocalizationSrv = rospy.Service("stop_sound_localization", Empty, self.handleUnsubscribeSoundLocalization)
        self.setEnergyComputationSrv = rospy.Service("set_energy_computation", SetEnergyComputation, self.handleSetEnergyComputation)
        self.setSensitivitySrv = rospy.Service("set_sensitivity", SetSensitivity, self.handleSetSensitivity)
        rospy.loginfo("naoqi_soundLocalization is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.soundLocalizationProxy = self.get_proxy("ALSoundLocalization")
        if self.memProxy is None or self.soundLocalizationProxy is None:
            exit(1)
    
    def handleSubscribeSoundLocalization(self, req):
        try:
            self.soundLocalizationProxy.subscribe("sound_localization")
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
    def handleUnsubscribeSoundLocalization(self, req):
        try:
            self.soundLocalizationProxy.unsubscribe("sound_localization")
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
    def handleSetEnergyComputation(self, req):
        try:
            res = SetEnergyComputationResponse()
            self.soundLocalizationProxy.setParameter("EnergyComputation", req.status)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            res.success = False
            return res
            
    def handleSetSensitivity(self, req):
        try:
            res = SetSensitivityResponse()
            self.soundLocalizationProxy.setParameter("Sensitivity", req.sensitivity)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            res.success = False
            return res

    def run(self):
        while self.is_looping():
            try:
                r = rospy.Rate(5)
                sound = self.memProxy.getData("ALSoundLocalization/SoundLocated")
                if (len(sound)) > 0 and sound != self.pre_sound_localization:
                    self.soundLocalization.header.stamp = rospy.get_rostime()
                    self.soundLocalization.azimuth = sound[1][0]
                    self.soundLocalization.elevation = sound[1][1]
                    self.soundLocalization.confidence = sound[1][2]
                    self.soundLocalization.energy = sound[1][3] 
                    self.soundLocalization.head_position_frame_torso.linear.x = sound[2][0]
                    self.soundLocalization.head_position_frame_torso.linear.y = sound[2][1]
                    self.soundLocalization.head_position_frame_torso.linear.z = sound[2][2]
                    self.soundLocalization.head_position_frame_torso.angular.x = sound[2][3]
                    self.soundLocalization.head_position_frame_torso.angular.y = sound[2][4]
                    self.soundLocalization.head_position_frame_torso.angular.z = sound[2][5]
                    self.soundLocalization.head_position_frame_robot.linear.x = sound[3][0]
                    self.soundLocalization.head_position_frame_robot.linear.y = sound[3][1]
                    self.soundLocalization.head_position_frame_robot.linear.z = sound[3][2]
                    self.soundLocalization.head_position_frame_robot.angular.x = sound[3][3]
                    self.soundLocalization.head_position_frame_robot.angular.y = sound[3][4]
                    self.soundLocalization.head_position_frame_robot.angular.z = sound[3][5]

                    self.soundLocalizationPub.publish(self.soundLocalization)
                    self.pre_sound_localization = sound
                    
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    soundLocalization = NaoqiSoundLocalization()
    soundLocalization.start()
    rospy.spin()
