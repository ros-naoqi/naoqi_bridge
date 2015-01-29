#!/usr/bin/env python
import rospy
from naoqi_driver.naoqi_moveto import MoveToListener

if __name__ == "__main__":
  moveto = MoveToListener()
  rospy.spin()
