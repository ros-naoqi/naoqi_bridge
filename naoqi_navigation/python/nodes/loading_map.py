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

class LoadingMap(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self, 'loading_map')
        self.navigation = None
        self.connectNaoQi()

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Node Loading map at %s:%d", self.pip, self.pport)

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

    def loadMap(self, pathtomap):
        '''Loading the map
        '''
        self.navigation.stopLocalization()
        res = self.navigation.loadExploration(pathtomap)
        if res:
          rospy.loginfo("The map %s has been loaded", pathtomap)
        else:
          rospy.logwarn("The map %s cannot be loaded", pathtomap)
        return res

    def relocalization(self, x, y, theta):
        '''Relocalization of the robot according to the position given as input
        Input  : The robot's position (x, y, theta)
        '''
        self.navigation.relocalizeInMap([x ,y ,theta])
        self.navigation.startLocalization()


if __name__ == '__main__':
    rospy.loginfo("LoadingMap based on Naoqi")
    pub = LoadingMap()

    #load the map and relocalize
    raw_input("Move Pepper to the position that corresponds to\
     zero in the map and press enter")
    pathtomap = rospy.get_param('~path_to_map', "")
    if pub.loadMap(pathtomap):
        pub.relocalization(0,0,0)

    exit(0)
