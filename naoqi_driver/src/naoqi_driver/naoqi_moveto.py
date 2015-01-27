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
import almath

class MoveToListener(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_moveto_listener')
        self.connectNaoQi()

        self.subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

    def callback(self, poseStamped):
        pose = Pose(poseStamped.pose.position, poseStamped.pose.orientation)
        quat = almath.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        rotation = almath.rotation3DFromQuaternion(quat)
        position3d = almath.Position3D(pose.position.x, pose.position.y, pose.position.z)
        worldToTarget = almath.Pose2D(position3d.x, position3d.y, rotation.wz)
        worldToRobot = almath.Pose2D(self.motionProxy.getRobotPosition(True))
        robotToTarget = almath.pinv(worldToRobot) * worldToTarget
        robotToTarget.theta = almath.modulo2PI(robotToTarget.theta)        
        self.motionProxy.moveTo(robotToTarget.x, robotToTarget.y, robotToTarget.theta)
