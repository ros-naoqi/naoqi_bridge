#!/usr/bin/env python

#
# ROS node to set Pepper's orthogonal security distance, tangential security 
# distance and read move faied information
# This code is currently compatible to NaoQI version 2.3
#
# Copyright 2015 Kanae Kochigami, The University of Tokyo
# http://wiki.ros.org/pepper
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy
import sys
import naoqi
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import ( ALModule, ALBroker, ALProxy )
from std_msgs.msg import String 
from std_srvs.srv import (
    EmptyResponse,
    Empty)
#from naoqi_bridge_msgs.srv import SetArmsEnabled
from naoqi_bridge_msgs.srv import ( TangentialSecurityDistance, OrthogonalSecurityDistance )

#
# Notes:
# - A port number > 0 for the module must be specified.
#   If port 0 is used, a free port will be assigned automatically,
#   but naoqi is unable to pick up the assigned port number, leaving
#   the module unable to communicate with naoqi (1.10.52).
#
# - Callback functions _must_ have a docstring, otherwise they won't get bound.
#
# - Shutting down the broker manually will result in a deadlock,
#   not shutting down the broker will sometimes result in an exception
#   when the script ends (1.10.52).
#

class NaoqiExternalCollisionAvoidance(ALModule):
    "Sends callbacks for bumper failed information to ROS"
    def __init__(self, moduleName):
        # get connection from command line:
        from optparse import OptionParser

        parser = OptionParser()
        parser.add_option("--ip", dest="ip", default="",
                          help="IP/hostname of broker. Default is system's default IP address.", metavar="IP")
        parser.add_option("--port", dest="port", default=0,
                          help="IP/hostname of broker. Default is automatic port.", metavar="PORT")
        parser.add_option("--pip", dest="pip", default="127.0.0.1",
                          help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
        parser.add_option("--pport", dest="pport", default=9559,
                          help="port of parent broker. Default is 9559.", metavar="PORT")

        (options, args) = parser.parse_args()
        self.ip = options.ip
        self.port = int(options.port)
        self.pip = options.pip
        self.pport = int(options.pport)
        self.moduleName = moduleName

        self.init_almodule()

        # ROS initialization:
        rospy.init_node('naoqi_externalCollisionAvoidance')

        # init. messages:
        self.moveFailed = String()
        
        self.setOrthogonalSecurityDistanceSrv = rospy.Service("naoqi_externalCollisionAvoidance/orthogonal_distance", OrthogonalSecurityDistance, self.handleOrthogonalSecurityDistanceSrv)
        self.setTangentialSecurityDistanceSrv = rospy.Service("naoqi_externalCollisionAvoidance/tangential_distance", TangentialSecurityDistance, self.handleTangentialSecurityDistanceSrv)
        self.moveFailedPub = rospy.Publisher("move_failed", String, queue_size=10)
        
        self.subscribe()

        rospy.loginfo("nao_externalCollisionAvoidance initialized")

    def init_almodule(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError,e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)

        self.memProxy = ALProxy("ALMemory", self.pip, self.pport)
        # TODO: check self.memProxy.version() for > 1.6
        if self.memProxy is None:
            rospy.logerror("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)
        self.motionProxy = ALProxy("ALMotion", self.pip, self.pport)


    def shutdown(self):
        self.unsubscribe()
        # Shutting down broker seems to be not necessary any more
        # try:
        #     self.broker.shutdown()
        # except RuntimeError,e:
        #     rospy.logwarn("Could not shut down Python Broker: %s", e)


    def subscribe(self):
        self.memProxy.subscribeToEvent("ALMotion/MoveFailed", self.moduleName, "onMoveFailed")
        

    def unsubscribe(self):
        self.memProxy.unsubscribeToEvent("ALMotion/MoveFailed", self.moduleName)

    def handleOrthogonalSecurityDistanceSrv(self, req):
        if (req.orthogonal_distance < 0.001 or req.orthogonal_distance == 0.001):
            return EmptyResponse
        self.motionProxy.setOrthogonalSecurityDistance(req.orthogonal_distance.data)
        return EmptyResponse

    def handleTangentialSecurityDistanceSrv(self, req):
        if (req.tangential_distance < 0.001 or req.tangential_distance == 0.001):
            return EmptyResponse
        self.motionProxy.setTangentialSecurityDistance(req.tangential_distance.data)
        return EmptyResponse

    def onMoveFailed(self, strVarName, value, strMessage):
        "Called when ALMotion/MoveFailed event is raised in ALMemory"
        publish_message = "Cause of move failed: " + value[0]
        if value[1] == 0:
            publish_message += ", Status: move not started" 
        if value[1] == 1:
            publish_message += ", Status: move started but stopped" 
    
        publish_message += ", Obstacle position: "+ str(value[2])
        self.moveFailed.data = publish_message;
        self.moveFailedPub.publish(self.moveFailed)
        rospy.logdebug("move failed: name=%s, value=%s, message=%s.", strVarName, value, strMessage);


if __name__ == '__main__':
    ROSNaoqiExternalCollisionAvoidanceModule = NaoqiExternalCollisionAvoidance("ROSNaoqiExternalCollisionAvoidanceModule")

    rospy.spin()

    rospy.loginfo("Stopping naoqi_externalCollisionAvoidance ...")
    ROSNaoqiExternalCollisionAvoidanceModule.shutdown();
    rospy.loginfo("naoqi_externalCollisionAvoidance stopped.")
    exit(0)
