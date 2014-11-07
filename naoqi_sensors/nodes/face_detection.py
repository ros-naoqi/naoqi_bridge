#!/usr/bin/env python
import rospy
from naoqi_sensors.naoqi_face_detection import NaoqiFaceDetection

if __name__ == "__main__":
  naoface = NaoqiFaceDetection()
  naoface.start()
  rospy.spin()
