#!/usr/bin/env python
__author__ = 'lsouchet'


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
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

#import NAO dependencies
from naoqi_driver.naoqi_node import NaoqiNode
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
import almath

class ParticlesPublisher(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_localization')
        self.navigation = None
        self.connectNaoQi()
        self.publishers = {}
        self.publishers["uncertainty"] = rospy.Publisher('localization_uncertainty', Marker)
        self.publishers["map_tf"] = rospy.Publisher('/tf', TFMessage, latch=True)
        self.publishers["map"] = rospy.Publisher('naoqi_exploration_map', OccupancyGrid , queue_size=None)
        self.publishers["exploration_path"] = rospy.Publisher('naoqi_exploration_path', Path, queue_size=None)
        self.frequency = 2
        self.rate = rospy.Rate(self.frequency)

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Exploration Node Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.navigation = self.get_proxy("ALNavigation")
        self.motion = self.get_proxy("ALMotion")
        if self.navigation is None or self.motion is None:
            rospy.logerr("Unable to reach ALMotion and ALNavigation.")
            exit(0)
        version_array = self.get_proxy("ALSystem").systemVersion().split('.')
        if len(version_array) < 3:
            rospy.logerr("Unable to deduce the system version.")
            exit(0)
        version_tuple = (int(version_array[0]), int(version_array[1]), int(version_array[2]))
        min_version = (2, 5, 2)
        if version_tuple < min_version:
            rospy.logerr("Naoqi version " + str(min_version) + " required for localization. Yours is " + str(version_tuple))
            exit(0)


    def get_ros_quaternion(self, almath_quaternion):
        output = Quaternion()
        output.w = almath_quaternion.w
        output.x = almath_quaternion.x
        output.y = almath_quaternion.y
        output.z = almath_quaternion.z
        return output

    def get_navigation_tf(self, navigation_pose):
        navigation_tf = TransformStamped()
        navigation_tf.header.frame_id = "/map"
        navigation_tf.header.stamp = rospy.Time.now()
        navigation_tf.child_frame_id = "/odom"
        navigation_tf.transform.translation .x = navigation_pose.x
        navigation_tf.transform.translation .y = navigation_pose.y
        navigation_tf.transform.rotation = self.get_ros_quaternion(
                    almath.Quaternion_fromAngleAndAxisRotation(navigation_pose.theta, 0, 0, 1))
        return navigation_tf

    def build_laser_scan(self, ranges):
        result = LaserScan()
        result.header.stamp = rospy.Time.now()
        result.angle_min = -almath.PI
        result.angle_max = almath.PI
        if len(ranges[1]) > 0:
            result.angle_increment = (result.angle_max - result.angle_min) / len(ranges[1])
        result.range_min = 0.0
        result.range_max = ranges[0]
        for range_it in ranges[1]:
            result.ranges.append(range_it[1])
        return result

    # do it!
    def run(self):
        while self.is_looping():
            navigation_tf_msg = TFMessage()
            try:
                motion_to_robot = almath.Pose2D(self.motion.getRobotPosition(True))
                localization = self.navigation.getRobotPositionInMap()
                exploration_path = self.navigation.getExplorationPath()
            except Exception as e:
                navigation_tf_msg.transforms.append(self.get_navigation_tf(almath.Pose2D()))
                self.publishers["map_tf"].publish(navigation_tf_msg)
                self.rate.sleep()
                continue
            if len(localization) > 0 and len(localization[0]) == 3:
                navigation_to_robot = almath.Pose2D(localization[0][0], localization[0][1], localization[0][2])
                navigation_to_motion = navigation_to_robot * almath.pinv(motion_to_robot)
                navigation_tf_msg.transforms.append(self.get_navigation_tf(navigation_to_motion))
            self.publishers["map_tf"].publish(navigation_tf_msg)
            if len(localization) > 0:
                if self.publishers["uncertainty"].get_num_connections() > 0:
                    uncertainty = Marker()
                    uncertainty.header.frame_id = "/base_footprint"
                    uncertainty.header.stamp = rospy.Time.now()
                    uncertainty.type = Marker.CYLINDER
                    uncertainty.scale = Vector3(localization[1][0], localization[1][1], 0.2)
                    uncertainty.pose.position = Point(0, 0, 0)
                    uncertainty.pose.orientation = self.get_ros_quaternion(
                        almath.Quaternion_fromAngleAndAxisRotation(0, 0, 0, 1))
                    uncertainty.color = ColorRGBA(1, 0.5, 0.5, 0.5)
                    self.publishers["uncertainty"].publish(uncertainty)
            if self.publishers["map"].get_num_connections() > 0:
                aggregated_map = None
                try:
                    aggregated_map = self.navigation.getMetricalMap()
                except Exception as e:
                    print("error " + str(e))
                if aggregated_map is not None:
                    map_marker = OccupancyGrid()
                    map_marker.header.stamp = rospy.Time.now()
                    map_marker.header.frame_id = "/map"
                    map_marker.info.resolution = aggregated_map[0]
                    map_marker.info.width = aggregated_map[1]
                    map_marker.info.height = aggregated_map[2]
                    map_marker.info.origin.orientation = self.get_ros_quaternion(
                        almath.Quaternion_fromAngleAndAxisRotation(-1.57, 0, 0, 1))
                    map_marker.info.origin.position.x = aggregated_map[3][0]
                    map_marker.info.origin.position.y = aggregated_map[3][1]
                    map_marker.data = aggregated_map[4]
                    self.publishers["map"].publish(map_marker)
            if self.publishers["exploration_path"].get_num_connections() > 0:
                path = Path()
                path.header.stamp = rospy.Time.now()
                path.header.frame_id = "/map"
                for node in exploration_path:
                    current_node = PoseStamped()
                    current_node.pose.position.x = node[0]
                    current_node.pose.position.y = node[1]
                    path.poses.append(current_node)
                self.publishers["exploration_path"].publish(path)
            self.rate.sleep()

if __name__ == '__main__':
    pub = ParticlesPublisher()
    pub.start()

    rospy.spin()
    exit(0)
