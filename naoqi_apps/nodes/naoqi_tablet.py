#!/usr/bin/env python

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import (
    EmptyResponse,
    Empty,
    TriggerResponse,
    Trigger)
from naoqi_bridge_msgs.srv import ( 
    SetStringResponse,
    SetString,
    GetStringResponse,
    GetString,
    ConfigureWifiResponse,
    ConfigureWifi)

class NaoqiTablet(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_tablet')
        self.connectNaoQi()
        self.image_folder = "img/"
        self.showImageSrv = rospy.Service("show_image", SetString, self.handleShowImageSrv)
        self.hideImageSrv = rospy.Service("hide_image", Empty, self.handleHideImageSrv)
        self.showAppSrv = rospy.Service("show_app", SetString, self.handleShowAppSrv)
        self.showWebViewSrv = rospy.Service("show_webview", SetString, self.handleShowWebviewSrv)
        self.setImagePathSrv = rospy.Service("set_show_image_folder_path", SetString, self.handleSetFolderPathSrv)
        self.getImagePathSrv = rospy.Service("get_show_image_folder_path", GetString, self.handleGetFolderPathSrv)
        self.configureWifiSrv = rospy.Service("configure_wifi", ConfigureWifi, self.handleConfigureWifiSrv)
        self.connectWifiSrv = rospy.Service("connect_wifi", SetString, self.handleConnectWifiSrv)
        self.disconnectWifiSrv = rospy.Service("disconnect_wifi", Trigger, self.handleDisconnectWifiSrv)
        self.enableWifiSrv = rospy.Service("enable_wifi", Empty, self.handleEnableWifiSrv)
        self.disableWifiSrv = rospy.Service("disable_wifi", Empty, self.handleDisableWifiSrv)
        self.forgetWifiSrv = rospy.Service("forget_wifi", SetString, self.handleForgetWifiSrv)
        self.getWifiStatusSrv = rospy.Service("get_wifi_status", GetString, self.handleGetWifiStatusSrv)
        rospy.loginfo("naoqi_tablet initialized")
    
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.tabletProxy = self.get_proxy("ALTabletService")
        if self.tabletProxy is None:
            rospy.logerr("Could not get a proxy to ALTabletService")
            rospy.logerr("Please make sure that your robot has a tablet.")
            exit(1)

    def handleShowImageSrv(self, req):
        try:
            image_path = "http://198.18.0.1/apps/"+ self.image_folder + str(req.data)
            res = SetStringResponse()
            if self.tabletProxy.showImage(image_path) == True:
                rospy.loginfo("Showing image: " + str(req.data))
                res.success = True
               
            else:
                rospy.loginfo("Please confirm the file name and " + str(req.data) + " really exists under /home/nao/.local/share/PackageManager/apps/" + self.image_folder + "html.")
                res.success.data = False
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
        
    def handleHideImageSrv(self, req):
        try:
            self.tabletProxy.hideImage()
            rospy.loginfo("Reset image on the tablet")
            return EmptyResponse()
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleShowAppSrv(self, req):
        try:
            res = SetStringResponse()
            res.success = False
            if self.tabletProxy.loadApplication(req.data):
                if self.tabletProxy.showWebview():
                    rospy.loginfo("Showing app: " + str(req.data))
                    res.success = True
                else:
                    rospy.loginfo("Showing webview failed")
            else:
                rospy.loginfo("Please confirm your app name: " + req.data + "really exists.")
                rospy.loginfo("In addtion, be sure that an index.html should be in a html folder in an app folder. (ex: /home/nao/.local/share/PackageManager/apps/<app>/html/index.html)")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleShowWebviewSrv(self, req):
        try:
            res = SetStringResponse()
            res.success = False
            if self.tabletProxy.showWebview(req.data):
                rospy.loginfo("Trying to show web page: " + str(req.data))
                rospy.loginfo("If there is nothing appeared, please confirm the url '" + str(req.data) + "' really exists.")
                res.success = True
            else:
                rospy.loginfo("Showing webview failed")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetFolderPathSrv(self, req):
        res = SetStringResponse()
        try:
            self.image_folder = req.data
            if self.image_folder[0]=="/":
                self.image_folder = self.image_folder.strip("/")
            if self.image_folder[len(self.image_folder)-1] != "/":
                self.image_folder += "/"
            res.success = True
            rospy.loginfo("ALTabletService: ShowImage will show pictures under /home/nao/.local/share/PackageManager/apps/" + self.image_folder + "html/." )
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleGetFolderPathSrv(self, req):
        try:
            res = GetStringResponse()
            res.data = "/home/nao/.local/share/PackageManager/apps/" + str(self.image_folder) + "html/"
            rospy.loginfo("ALTabletService: ShowImage will show pictures under /home/nao/.local/share/PackageManager/apps/" + self.image_folder + "html.")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleConfigureWifiSrv(self, req):
        try:
            res = ConfigureWifiResponse()
            res.success = self.tabletProxy.configureWifi(req.security, req.ssid, req.key)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleConnectWifiSrv(self, req):
        try:
            res = SetStringResponse()
            res.success = self.tabletProxy.connectWifi(req.data)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleDisconnectWifiSrv(self, req):
        try:
            res = TriggerResponse()
            res.success = self.tabletProxy.disconnectWifi()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleEnableWifiSrv(self, req):
        try:
            res = EmptyResponse()
            self.tabletProxy.enableWifi()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleDisableWifiSrv(self, req):
        try:
            res = EmptyResponse()
            self.tabletProxy.disableWifi()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleForgetWifiSrv(self, req):
        try:
            res = SetStringResponse()
            res.success = self.tabletProxy.forgetWifi(req.data)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetWifiStatusSrv(self, req):
        try:
            res = GetStringResponse()
            res.data = self.tabletProxy.getWifiStatus()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

if __name__ == '__main__':
    tablet = NaoqiTablet()
    rospy.spin()
