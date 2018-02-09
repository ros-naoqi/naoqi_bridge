#!/usr/bin/env python

#                                                                             
#  Copyright 2015 Aldebaran                                                   
#                                                                             
#  Licensed under the Apache License, Version 2.0 (the "License");            
#  you may not use this file except in compliance with the License.           
#  You may obtain a copy of the License at                                    
#                                                                             
#      http://www.apache.org/licenses/LICENSE-2.0                             
#                                                                             
#  Unless required by applicable law or agreed to in writing, software        
#  distributed under the License is distributed on an "AS IS" BASIS,          
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   
#  See the License for the specific language governing permissions and        
#  limitations under the License.                                             
#                                                                             
# 
import rospy
from naoqi_bridge_msgs.msg import TactileTouch, Bumper, HandTouch
from std_msgs.msg import Bool
from naoqi_driver.naoqi_node import NaoqiNode

class NaoqiTactile(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_tactile')
        self.connectNaoQi()

        # key messages
        self.keys = ["FrontTactilTouched", "MiddleTactilTouched", "RearTactilTouched", "RightBumperPressed", "LeftBumperPressed", "HandRightBackTouched", "HandRightLeftTouched", "HandRightRightTouched", "HandLeftBackTouched", "HandLeftLeftTouched", "HandLeftRightTouched"]
        self.previousState = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.previousBackBumperPressed = Bumper.stateReleased
        self.previousFootContactState = False
        self.nameOfSensors = [TactileTouch.buttonFront, TactileTouch.buttonMiddle, TactileTouch.buttonRear, Bumper.right, Bumper.left, HandTouch.RIGHT_BACK, HandTouch.RIGHT_LEFT, HandTouch.RIGHT_RIGHT, HandTouch.LEFT_BACK, HandTouch.LEFT_LEFT, HandTouch.LEFT_RIGHT]        
        self.bumperBackButton = Bumper.back
        
        # init. messages:
        self.tactile = TactileTouch()
        self.bumper = Bumper()
        self.hand = HandTouch()

        self.tactilePub = rospy.Publisher("tactile_touch", TactileTouch, queue_size=10)
        self.bumperPub = rospy.Publisher("bumper", Bumper, queue_size=10)
        self.handPub = rospy.Publisher("hand_touch", HandTouch, queue_size=10)
        
        try:
            BackBumperPressed = self.memProxy.getData("BackBumperPressed", 0)
        except RuntimeError:
            BackBumperPressed = None

        if BackBumperPressed is None:
            self.hasBackBumperPressedKey = False
            rospy.loginfo("Foot contact key is not present in ALMemory, will not publish to foot_contact topic.")
        else:
            self.hasBackBumperPressedKey = True
            self.previousBackBumperPressed = Bumper.statePressed
          
        try:
            footContact = self.memProxy.getData("footContact", 0)
        except RuntimeError:
            footContact = None

        if footContact is None:
            self.hasFootContactKey = False
            rospy.loginfo("Foot contact key is not present in ALMemory, will not publish to foot_contact topic.")
        else:
            self.hasFootContactKey = True
            self.previousFootContact = True
            self.footContactPub = rospy.Publisher("foot_contact", Bool, latch=True, queue_size=10)
            self.footContactPub.publish(footContact > 0.0)
        rospy.loginfo("naoqi_tactile initialized")

    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        if self.memProxy is None:
            exit(1)

    def run(self):
        while self.is_looping():
            try:
                for i in range(len(self.keys)):
                    if self.memProxy.getData(self.keys[i]) != self.previousState[i]:
                        if i >= 0 and i < 3:
                            self.tactile.button = self.nameOfSensors[i]
                            self.tactile.state = self.memProxy.getData(self.keys[i])
                            self.previousState[i] = self.tactile.state
                            self.tactilePub.publish(self.tactile)

                        elif i >= 3 and i < 5:
                            self.bumper.bumper = self.nameOfSensors[i]
                            self.bumper.state = self.memProxy.getData(self.keys[i])
                            self.previousState[i] = self.bumper.state
                            self.bumperPub.publish(self.bumper)
                            
                        else:
                            self.hand.hand = self.nameOfSensors[i]
                            self.hand.state = self.memProxy.getData(self.keys[i])
                            self.previousState[i] = self.hand.state
                            self.handPub.publish(self.hand)
         
                if self.hasBackBumperPressedKey:
                    if self.memProxy.getData("BackBumperPressed") != self.previousBackBumperPressed:
                        self.bumper.bumper = self.bumperBackButton
                        self.bumper.state = self.memProxy.getData("BackBumperPressed")
                        self.previousBackBumperPressed = self.bumper.state
                        self.bumperPub.publish(self.bumper)
                
                if self.hasFootContactKey:
                    if self.memProxy.getData("footContact") != self.previousFootContactState:
                        rospy.loginfo("foot changed")
                        self.previousFootContactState = self.memProxy.getData("footContact")
                        self.footContactPub.publish(self.memProxy.getData("footContact"))

            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    tactile = NaoqiTactile()
    tactile.start()
    rospy.spin()
