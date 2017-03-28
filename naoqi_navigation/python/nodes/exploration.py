#!/usr/bin/env python
# Copyright (C) 2017 SoftBank Robotics Europe
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#import ROS dependencies
import rospy

#import NAO dependencies
from naoqi_driver.naoqi_node import NaoqiNode

class Exploration(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_exploration')
        self.navigation = None
        self.connectNaoQi()

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        '''Connect to Naoqi modules
        '''
        rospy.loginfo("Exploration Node Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.navigation = self.get_proxy("ALNavigation")
        if self.navigation is None:
            rospy.logerr("Unable to reach ALNavigation.")
            exit(0)

        version_array = self.get_proxy("ALSystem").systemVersion().split('.')
        if len(version_array) < 3:
            rospy.logerr("Unable to deduce the system version.")
            exit(0)
        version_tuple = (int(version_array[0]), int(version_array[1]), int(version_array[2]))
        min_version = (2, 5, 2)
        if version_tuple < min_version:
            rospy.logerr("Naoqi version " + str(min_version) +
            " required for localization. Yours is " + str(version_tuple))
            exit(0)

        self.tts  = self.get_proxy("ALTextToSpeech")

        self.motion = self.get_proxy("ALMotion")
        if self.motion is not None:
            if not self.motion.robotIsWakeUp():
                self.motion.wakeUp()

    def learnMap(self, radius):
        '''Learning a new map and saving it
        Input  : maximal radius to explore (in meters)
        '''
        #exploration
        if self.tts is not None:
            self.tts.post.say(str("starting exploration"))

        rospy.loginfo("Starting exploration in a radius of %d m", radius)

        self.setThresholds(0.2, 0.08)

        self.navigation.explore(radius)

        self.navigation.stopExploration()
        rospy.loginfo("Exploration has finished")

        #saving exploration
        self.pathtomap = self.navigation.saveExploration()
        rospy.loginfo("Path to the map: %s", self.pathtomap)

        if self.tts is not None:
            self.tts.post.say(str("saving the map"))

        self.setThresholds(0.4, 0.1)

    def setThresholds(self, th_orth, th_tan):
        '''Set safety thresholds
        Input th_orth : orthogonal security distance
        Input th_tan : tangential security distance
        '''
        if self.motion is not None:
            self.motion.setOrthogonalSecurityDistance(th_orth)
            self.motion.setTangentialSecurityDistance(th_tan)

if __name__ == '__main__':
    rospy.loginfo("Exploration based on Naoqi")
    pub = Exploration()

    radius = rospy.get_param('~radius', 10)
    pub.learnMap(radius)

    exit(0)
