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
from naoqi_sensors.naoqi_sonar import SonarSensor, SonarPublisher

if __name__ == '__main__':
    sonar_params = rospy.get_param('sonar_devices', [{'al_memory_key': 'Device/SubDeviceList/US/Left/Sensor/Value',
                                                      'frame_id': 'LSonar_frame',
                                                      'topic_name': '~/nao/sonar_left'},
                                                     {'al_memory_key': 'Device/SubDeviceList/US/Right/Sensor/Value',
                                                      'frame_id': 'RSonar_frame',
                                                      'topic_name': '~/nao/sonar_right'}])
    sonars = []
    for param in sonar_params:
        print("{}: {} {}".format(param['al_memory_key'], param['frame_id'],param['topic_name']))
        sonar = SonarSensor(param['al_memory_key'],     # AL memory key
                            param['frame_id'],          # ROS frame_id
                            param['topic_name'])        # ROS topic to publish
        sonars.append(sonar)
    sonars = tuple(sonars)

    publisher = SonarPublisher( sonars )                      # list of sonars
    publisher.start()

    rospy.spin()
    exit(0)
