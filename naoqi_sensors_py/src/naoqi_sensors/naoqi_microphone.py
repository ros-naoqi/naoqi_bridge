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

from collections import defaultdict
import rospy

from naoqi_driver.naoqi_node import NaoqiNode

from dynamic_reconfigure.server import Server
from naoqi_sensors_py.cfg import NaoqiMicrophoneConfig
from naoqi_bridge_msgs.msg import AudioBuffer

from naoqi import ALModule, ALBroker, ALProxy

class NaoqiMic (ALModule, NaoqiNode):
    def __init__(self, name):
        NaoqiNode.__init__(self, name)
        self.myBroker = ALBroker("pythonBroker", "0.0.0.0", 0, self.pip, self.pport)
        ALModule.__init__(self, name)

        self.isSubscribed = False
        self.microVersion = 0

        # Create a proxy to ALAudioDevice
        try:
            self.audioProxy = self.get_proxy("ALAudioDevice")
        except Exception, e:
            rospy.logerr("Error when creating proxy on ALAudioDevice:")
            rospy.logerr(str(e))
            exit(1)

        try:
            self.robotProxy = self.get_proxy("ALRobotModel")
            self.microVersion = self.robotProxy._getMicrophoneConfig()
        except Exception, e:
            rospy.logwarn("Could not retrieve microphone version:")
            rospy.logwarn(str(e))
            rospy.logwarn("Microphone channel map might not be accurate.")

        def returnNone():
            return None
        self.config = defaultdict(returnNone)

        # ROS publishers
        self.pub_audio_ = rospy.Publisher('~audio_raw', AudioBuffer)

        # initialize the parameter server
        self.srv = Server(NaoqiMicrophoneConfig, self.reconfigure)

    def reconfigure( self, new_config, level ):
        """
        Reconfigure the microphones
        """
        rospy.loginfo('reconfigure changed')
        if self.pub_audio_.get_num_connections() == 0:
           rospy.loginfo('Changes recorded but not applied as nobody is subscribed to the ROS topics.')
           self.config.update(new_config)
           return self.config

        # check if we are already subscribed
        if not self.isSubscribed:
            rospy.loginfo('subscribed to audio proxy, since this is the first listener')
            self.audioProxy.setClientPreferences(self.getName(), new_config['frequency'], 0, 0)
            self.audioProxy.subscribe(self.getName())
            self.isSubscribed = True

        self.config.update(new_config)

        return self.config

    def run(self):
        r=rospy.Rate(2)
        while self.is_looping():
            if self.pub_audio_.get_num_connections() == 0:
                if self.isSubscribed:
                    rospy.loginfo('Unsubscribing from audio bridge as nobody listens to the topics.')
                    self.release()
                continue

            if not self.isSubscribed:
                self.reconfigure(self.config, 0)

            r.sleep()

        if self.isSubscribed:
            self.release()
        self.myBroker.shutdown()

    def release(self):
        self.audioProxy.unsubscribe(self.name)
        self.isSubscribed=False

    def processRemote(self, nbOfInputChannels, fNbOfInputSamples, timeStamp, inputBuff):
        audio_msg = AudioBuffer()

        # Deal with the sound
        # get data directly with the _getInputBuffer() function because inputBuff is corrupted in python
        mictmp = []
        for i in range (0,len(inputBuff)/2) :
            mictmp.append(ord(inputBuff[2*i])+ord(inputBuff[2*i+1])*256)

        # convert 16 bit samples to signed 16 bits samples
        for i in range (0,len(mictmp)) :
            if mictmp[i]>=32768 :
                mictmp[i]=mictmp[i]-65536

        if self.config['use_ros_time']:
            audio_msg.header.stamp = rospy.Time.now()
        else:
            audio_msg.header.stamp = rospy.Time(timeStamp)

        audio_msg.frequency = self.config['frequency']
        if self.microVersion == 0:
            channels = [0,2,1,4]
        else:
            channels = [3,5,0,2]

        audio_msg.channelMap = channels

        audio_msg.data = mictmp
        self.pub_audio_.publish(audio_msg)
