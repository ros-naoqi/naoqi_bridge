#!/usr/bin/env python

# Copyright (C) 2014 Aldebaran Robotics
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
from visualization_msgs.msg import Marker
from naoqi_bridge_msgs.msg import PoseWithConfidenceStamped

#import NAO dependencies
from naoqi_driver.naoqi_node import NaoqiNode
import almath


class PodPublisher(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_pod_node')

        self.markerpublisher = rospy.Publisher('naoqi_pod_publisher', PoseWithConfidenceStamped, queue_size=5)
        self.rate = rospy.Rate(2)
        self.init = False
        self.connectNaoQi()

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.rechargeProxy = self.get_proxy("ALRecharge")
        if self.rechargeProxy is not None:
            self.init = True

    # do it!
    def run(self):
        while self.is_looping():
            if not self.init:
                break
            if self.markerpublisher.get_num_connections() > 0:
                worldToPod = almath.Pose2D(self.rechargeProxy.getStationPosition())
                confidence = self.rechargeProxy._getConfidenceIndex()
                self.marker = PoseWithConfidenceStamped()
                self.marker.header.stamp = rospy.Time.now()
                self.marker.header.frame_id = "/odom"
                self.marker.pose.orientation = almath.Quaternion_fromAngleAndAxisRotation(worldToPod.theta, 0, 0, 1)
                self.marker.pose.position.x = worldToPod.x
                self.marker.pose.position.y = worldToPod.y
                self.marker.pose.position.z = 0.0
                self.marker.confidence_index = confidence
                self.markerpublisher.publish(self.marker)
            self.rate.sleep()

if __name__ == '__main__':
    publisher = PodPublisher()
    publisher.start()
    rospy.spin()
    exit(0)