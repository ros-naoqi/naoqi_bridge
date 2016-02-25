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
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    EmptyResponse,
    Empty,)
from naoqi_bridge_msgs.srv import ( 
    ShowImageResponse,
    ShowImage,)

class NaoqiTablet(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_tablet')
        self.connectNaoQi()
        
        self.image_path = ""
        self.showImageSrv = rospy.Service("show_image", ShowImage, self.handleShowImageSrv)
        self.hideImageSrv = rospy.Service("hide_image", Empty, self.handleHideImageSrv)
        self.showWebViewSrv = rospy.Service("show_webview", ShowImage, self.handleShowWebviewSrv)
        rospy.loginfo("naoqi_tablet initialized")
    
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.tabletProxy = self.get_proxy("ALTabletService")
        if self.tabletProxy is None:
            exit(1)
        
    def handleShowImageSrv(self, req):
        try:
            self.image_path = "http://198.18.0.1/apps/img/" + str(req.file_name.data)
            res = ShowImageResponse()
            if self.tabletProxy.showImage(self.image_path) == True:
                rospy.loginfo("Ok, I'll show you " + str(req.file_name.data) + " !")
                res.status.data = True
               
            else:
                rospy.loginfo("Please confirm the file name and " + str(req.file_name.data) + " really exists under /home/nao/.local/share/PackageManager/apps/img/html.")
                res.status.data = False
            self.image_path = ""
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleHideImageSrv(self, req):
        try:
            self.tabletProxy.hideImage()
            rospy.loginfo("I'll hide the picture on the tablet.")
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleShowWebviewSrv(self, req):
        try:
            self.image_path = "http://198.18.0.1/apps/img/" + str(req.file_name.data)
            res = ShowImageResponse()
            if self.tabletProxy.showWebview(self.image_path) == True:
                rospy.loginfo("Ok, I'll show you " + str(req.file_name.data) + " !")
                res.status.data = True
               
            else:
                rospy.loginfo("Please confirm the file name and " + str(req.file_name.data) + " really exists under /home/nao/.local/share/PackageManager/apps/img/html.")
                res.status.data = False
            self.image_path = ""
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
    def run(self):
        while self.is_looping():
            try:
                pass
            except RuntimeError, e:
                print "Error accessing ALMemory and ALMotion, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    tablet = NaoqiTablet()
    tablet.start()
    rospy.spin()
