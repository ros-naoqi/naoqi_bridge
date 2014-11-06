#!/usr/bin/env python
import rospy

from naoqi_sensors.naoqi_microphone import NaoqiMic

if __name__ == "__main__":
  ALToRosMics = NaoqiMic('ALToRosMics')
  ALToRosMics.start()
  rospy.spin()
