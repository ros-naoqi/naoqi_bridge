#!/usr/bin/env python
import rospy
from naoqi_sensors.naoqi_camera import NaoqiCam

if __name__ == "__main__":
  naocam = NaoqiCam()
  naocam.start()
  rospy.spin()
