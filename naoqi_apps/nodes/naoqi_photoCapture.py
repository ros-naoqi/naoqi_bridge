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
    TakePictureResponse,
    TakePicture,
    SetFolderPathResponse,
    SetFolderPath,
    GetFolderPath,
    GetFolderPathResponse,
    PhotoCaptureSetResolutionResponse,
    PhotoCaptureSetResolution,
    PhotoCaptureGetResolutionResponse,
    PhotoCaptureGetResolution,
    SetPictureFormatResponse,
    SetPictureFormat,
    GetPictureFormatResponse,
    GetPictureFormat
)

class NaoqiPhotoCapture(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_photoCapture')
        self.connectNaoQi()
        
        self.resolution = 2
        self.pictureFormat = "jpg"
        self.folderPath = "/home/nao/.local/share/PackageManager/apps/img/html/"
        self.fileName = "image"
        self.setResolutionSrv = rospy.Service("set_resolution", PhotoCaptureSetResolution, self.handleSetResolutionSrv)
        self.setPictureFormatSrv = rospy.Service("set_picture_format", SetPictureFormat, self.handleSetPictureFormat)
        self.setFolderPathSrv = rospy.Service("set_take_picture_folder_path", SetFolderPath, self.handleSetFolderPathSrv)
        self.takePictureSrv = rospy.Service("take_picture", TakePicture, self.handleTakePictureSrv)
        self.getResolutionSrv = rospy.Service("get_resolution", PhotoCaptureGetResolution, self.handleGetResolution) 
        self.getPictureFormatSrv = rospy.Service("get_picuture_format", GetPictureFormat, self.handleGetPictureFormat)
        self.getFolderPathSrv = rospy.Service("get_take_picture_folder_path", GetFolderPath, self.handleGetFolderPathSrv)
        rospy.loginfo("naoqi_photocapture initialized")
    
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.photoCaptureProxy = self.get_proxy("ALPhotoCapture")
        if self.photoCaptureProxy is None:
            exit(1)
        
    def handleSetResolutionSrv(self, req):
        try:
            self.resolution = req.name.data 
            self.photoCaptureProxy.setResolution(self.resolution)
            rospy.loginfo("ALPhotoCapture: Resolution is set to " + str(self.resolution))
            return PhotoCaptureSetResolutionResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleSetPictureFormat(self, req):
        try:
            self.pictureFormat = req.name.data
            if self.pictureFormat == 0:
                self.pictureFormat = "bmp"
            elif self.pictureFormat == 1:
                self.pictureFormat = "dib"
            elif self.pictureFormat == 2:
                self.pictureFormat = "jpeg"
            elif self.pictureFormat == 3:
                self.pictureFormat = "jpg"
            elif self.pictureFormat == 4:
                self.pictureFormat = "jpe"
            elif self.pictureFormat == 5:
                self.pictureFormat = "png"
            elif self.pictureFormat == 6:
                self.pictureFormat = "pbm"
            elif self.pictureFormat == 7:
                self.pictureFormat = "pgm"
            elif self.pictureFormat == 8:
                self.pictureFormat = "ppm"
            elif self.pictureFormat == 9:
                self.pictureFormat = "sr"
            elif self.pictureFormat == 10:
                self.pictureFormat = "ras"
            elif self.pictureFormat == 11:
                self.pictureFormat = "tiff"
            elif self.pictureFormat == 12:
                self.pictureFormat = "tif"
            self.photoCaptureProxy.setPictureFormat(self.pictureFormat)
            rospy.loginfo("ALPhotoCapture: Picture Format is set to " + self.pictureFormat)
            return SetPictureFormatResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleSetFolderPathSrv(self, req):
        try:
            self.folderPath = req.folder_path.data
            rospy.loginfo("ALPhotoCapture: Picture Folder Path is set to " + self.folderPath)
            return SetFolderPathResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleTakePictureSrv(self, req):
        try:
            self.fileName = req.picture_name.data 
            res = TakePictureResponse()
            val = None
            val = self.photoCaptureProxy.takePicture(self.folderPath, self.fileName)
            if val is None:
                res.status.data = False
                rospy.loginfo("ALPhotoCapture: Failed in storing '" + self.fileName + "' in" + self.folderPath)
            else:
                res.status.data = True
                rospy.loginfo("ALPhotoCapture: New Picture '" + self.fileName + "' is stored in" + self.folderPath)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleGetResolution(self, req):
        try:
            res = PhotoCaptureGetResolutionResponse()
            res.name.data = self.photoCaptureProxy.getResolution()
            rospy.loginfo("ALPhotoCapture: Resolution is set to " + str(res.name.data))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetPictureFormat(self, req):
        try:
            res = GetPictureFormatResponse()
            res.name.data = self.photoCaptureProxy.getPictureFormat()
            rospy.loginfo("ALPhotoCapture: Picture Format is set to " + res.name.data)
            if res.name.data == "bmp":
                res.name.data = 0
            elif res.name.data == "dib":
                res.name.data = 1
            elif res.name.data == "jpeg":
                res.name.data = 2
            elif res.name.data == "jpg":
                res.name.data = 3
            elif res.name.data == "jpe":
                res.name.data = 4
            elif res.name.data == "png":
                res.name.data = 5
            elif res.name.data == "pbm":
                res.name.data = 6
            elif res.name.data == "pgm":
                res.name.data = 7
            elif res.name.data == "ppm":
                res.name.data = 8
            elif res.name.data == "sr":
                res.name.data = 9
            elif res.name.data == "ras":
                res.name.data = 10
            elif res.name.data == "tiff":
                res.name.data = 11
            elif res.name.data == "tif":
                res.name.data = 12
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetFolderPathSrv(self, req):
        try:
            res = GetFolderPathResponse()
            res.folder_path.data = self.folderPath
            rospy.loginfo("ALPhotoCapture: Picture Folder Path is set to " + self.folderPath)
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
    photocapture = NaoqiPhotoCapture()
    photocapture.start()
    rospy.spin()
