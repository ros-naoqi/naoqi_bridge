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

#import NAO dependencies
from naoqi_driver.naoqi_node import NaoqiNode
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import almath
import tf
from tf.transformations import euler_from_quaternion

class MoveToListener(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_moveto_listener')
        self.connectNaoQi()
        self.listener = tf.TransformListener()

        self.goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goalCB)
        self.cvel_sub = rospy.Subscriber('cmd_vel', Twist, self.cvelCB)


    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

    def goalCB(self, poseStamped):
        # reset timestamp because of bug: https://github.com/ros/geometry/issues/82
        poseStamped.header.stamp = rospy.Time(0)
        try:
            robotToTarget1 = self.listener.transformPose("/base_footprint", poseStamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error while transforming pose: %s", str(e))
            return
        quat = robotToTarget1.pose.orientation
        (roll,pitch,yaw) = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        self.motionProxy.moveTo(robotToTarget1.pose.position.x, robotToTarget1.pose.position.y, yaw)

    def cvelCB(self, twist):
        rospy.logdebug("cmd_vel: %f %f %f", twist.linear.x, twist.linear.y, twist.angular.z)
        eps = 1e-3 # maybe 0,0,0 is a special command in motionProxy...
        if abs(twist.linear.x)<eps and abs(twist.linear.y)<eps and abs(twist.angular.z)<eps:
            self.motionProxy.moveToward(0,0,0)
        else:
            self.motionProxy.moveToward(twist.linear.x, twist.linear.y, twist.angular.z)
