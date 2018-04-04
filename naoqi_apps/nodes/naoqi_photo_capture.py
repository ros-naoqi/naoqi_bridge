#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi_bridge_msgs.srv import ( 
    SetStringResponse,
    SetString,
    SetFloatResponse,
    SetFloat,
    GetStringResponse,
    GetString,
    GetFloatResponse,
    GetFloat    
)

class NaoqiPhotoCapture(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_photo_capture')
        self.connectNaoQi()
        
        self.folderPath = "/home/nao/.local/share/PackageManager/apps/img/html/"
        self.setResolutionSrv = rospy.Service("set_resolution", SetFloat, self.handleSetResolutionSrv)
        self.setPictureFormatSrv = rospy.Service("set_picture_format", SetString, self.handleSetPictureFormat)
        self.setFolderPathSrv = rospy.Service("set_take_picture_folder_path", SetString, self.handleSetFolderPathSrv)
        self.takePictureSrv = rospy.Service("take_picture", SetString, self.handleTakePictureSrv)
        self.getResolutionSrv = rospy.Service("get_resolution", GetFloat, self.handleGetResolution) 
        self.getPictureFormatSrv = rospy.Service("get_picuture_format", GetString, self.handleGetPictureFormat)
        self.getFolderPathSrv = rospy.Service("get_take_picture_folder_path", GetString, self.handleGetFolderPathSrv)
        rospy.loginfo("naoqi_photo_capture initialized")
    
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.photoCaptureProxy = self.get_proxy("ALPhotoCapture")
        if self.photoCaptureProxy is None:
            exit(1)
        
    def handleSetResolutionSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.photoCaptureProxy.setResolution(int(req.data))
            rospy.loginfo("Camera Resolution is set to " + str(int(req.data)))
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleSetPictureFormat(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.photoCaptureProxy.setPictureFormat(req.data)
            res.success = True
            rospy.loginfo("Picture Format is set to " + req.data)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleSetFolderPathSrv(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.folderPath = req.data
            rospy.loginfo("Picture Folder Path is set to " + self.folderPath)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleTakePictureSrv(self, req):
        res = SetStringResponse()
        res.success = False
        try:
            self.photoCaptureProxy.takePicture(self.folderPath, req.data)
            if len(req.data) == 0:
                rospy.loginfo("Take Picture Failed: Please set file name")
            else:
                res.success = True
                rospy.loginfo("New Picture '" + req.data + "' is stored in" + self.folderPath)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleGetResolution(self, req):
        res = GetFloatResponse()
        try:
            res.data = self.photoCaptureProxy.getResolution()
            rospy.loginfo("Camera Resolution is set to " + str(res.data))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetPictureFormat(self, req):
        res = GetStringResponse()
        try:
            res.data = self.photoCaptureProxy.getPictureFormat()
            rospy.loginfo("Picture Format is set to " + res.data)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetFolderPathSrv(self, req):
        try:
            res = GetStringResponse()
            res.data = self.folderPath
            rospy.loginfo("Picture Folder Path is set to " + self.folderPath)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
if __name__ == '__main__':
    photocapture = NaoqiPhotoCapture()
    rospy.spin()
