#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    SetBoolResponse,
    SetBool,
    TriggerResponse,
    Trigger
)

from naoqi_bridge_msgs.srv import ( 
    SetStringResponse,
    SetString,
    SetFloatResponse,
    SetFloat,
    GetStringResponse,
    GetString,
    GetFloatResponse,
    GetFloat,
    TakePicturesResponse,
    TakePictures
)

class NaoqiPhotoCapture(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_photo_capture')
        self.connectNaoQi()
        
        self.folderPath = "/home/nao/.local/share/PackageManager/apps/img/html/"
        self.setCameraIDSrv = rospy.Service("set_camera_id", SetFloat, self.handleSetCameraIDSrv)
        self.setCaptureIntervalSrv = rospy.Service("set_capture_interval", SetFloat, self.handleSetCaptureIntervalSrv)
        self.setColorSpaceSrv = rospy.Service("set_color_space", SetFloat, self.handleSetColorSpaceSrv)
        self.setPictureFormatSrv = rospy.Service("set_picture_format", SetString, self.handleSetPictureFormat)
        self.setResolutionSrv = rospy.Service("set_resolution", SetFloat, self.handleSetResolutionSrv)
        self.setHalfPressedSrv = rospy.Service("set_half_pressed", SetBool, self.handleSetHalfPressedSrv)
        self.setFolderPathSrv = rospy.Service("set_take_picture_folder_path", SetString, self.handleSetFolderPathSrv)
        self.takePictureSrv = rospy.Service("take_picture", SetString, self.handleTakePictureSrv)
        self.takePicturesSrv = rospy.Service("take_pictures", TakePictures, self.handleTakePicturesSrv)
        self.getCameraIDSrv = rospy.Service("get_camera_id", GetFloat, self.handleGetCameraIDSrv)
        self.getCaptureIntervalSrv = rospy.Service("get_capture_interval", GetFloat, self.handleGetCaptureIntervalSrv)
        self.getColorSpaceSrv = rospy.Service("get_color_space", GetFloat, self.handleGetColorSpaceSrv)
        self.getResolutionSrv = rospy.Service("get_resolution", GetFloat, self.handleGetResolution) 
        self.getPictureFormatSrv = rospy.Service("get_picuture_format", GetString, self.handleGetPictureFormat)
        self.isHalfPressedSrv = rospy.Service("is_half_pressed", Trigger, self.handleIsHalfPressedSrv)
        self.getFolderPathSrv = rospy.Service("get_take_picture_folder_path", GetString, self.handleGetFolderPathSrv)
        rospy.loginfo("naoqi_photo_capture initialized")
    
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.photoCaptureProxy = self.get_proxy("ALPhotoCapture")
        if self.photoCaptureProxy is None:
            exit(1)

    def handleSetCameraIDSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.photoCaptureProxy.setCameraID(int(req.data))
            rospy.loginfo("Camera ID is set to " + str(int(req.data)))
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetCaptureIntervalSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.photoCaptureProxy.setCaptureInterval(int(req.data))
            rospy.loginfo("Capture interval is set to " + str(int(req.data)))
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetColorSpaceSrv(self, req):
        res = SetFloatResponse()
        res.success = False
        try:
            self.photoCaptureProxy.setColorSpace(int(req.data))
            rospy.loginfo("Color Space is set to " + str(int(req.data)))
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

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
    
    def handleSetHalfPressedSrv(self, req):
        res = SetBoolResponse()
        res.success = False
        try:
            self.photoCaptureProxy.setHalfPressed(req.data)
            res.success = True
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

    def handleTakePicturesSrv(self, req):
        res = TakePicturesResponse()
        res.success = False
        try:
            self.photoCaptureProxy.takePictures(req.number, self.folderPath, req.name)
            if len(req.name) == 0:
                rospy.loginfo("Take Picture Failed: Please set file name")
            else:
                res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetCameraIDSrv(self, req):
        res = GetFloatResponse()
        try:
            res.data = self.photoCaptureProxy.getCameraID()
            rospy.loginfo("Camera ID is set to " + str(int(res.data)))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetCaptureIntervalSrv(self, req):
        res = GetFloatResponse()
        try:
            self.photoCaptureProxy.getCaptureInterval()
            rospy.loginfo("Capture interval is set to " + str(int(res.data)))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetColorSpaceSrv(self, req):
        res = GetFloatResponse()
        try:
            self.photoCaptureProxy.getColorSpace()
            rospy.loginfo("Color Space is set to " + str(int(res.data)))
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
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

    def handleIsHalfPressedSrv(self, req):
        try:
            res = TriggerResponse()
            res.success = self.photoCaptureProxy.isHalfPressed()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
            
if __name__ == '__main__':
    photocapture = NaoqiPhotoCapture()
    rospy.spin()
