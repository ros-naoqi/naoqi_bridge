#!/usr/bin/env python

#
# ROS node to provide access to the camera by wrapping NaoQI access (may not
# be the most efficient way...)
#
# Copyright 2012 Daniel Maier, University of Freiburg
# Copyright 2014 Aldebaran Robotics
# http://www.ros.org/wiki/nao
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

from collections import defaultdict
import rospy
from sensor_msgs.msg import Image, CameraInfo
from nao_driver.nao_driver_naoqi import NaoNode
import camera_info_manager
from sensor_msgs.msg._CameraInfo import CameraInfo

from dynamic_reconfigure.server import Server
from nao_camera.cfg import NaoCameraConfig

from naoqi import ALProxy


# import resolutions
from nao_camera.vision_definitions import k960p, k4VGA, kVGA, kQVGA, kQQVGA
# import color spaces
from nao_camera.vision_definitions import kYUV422ColorSpace, kYUVColorSpace, \
                    kRGBColorSpace, kBGRColorSpace, kDepthColorSpace
# import extra parameters
from nao_camera.vision_definitions import kCameraSelectID, kCameraAutoExpositionID, kCameraAecAlgorithmID, \
                  kCameraContrastID, kCameraSaturationID, kCameraHueID, kCameraSharpnessID, kCameraAutoWhiteBalanceID, \
                  kCameraExposureID, kCameraGainID, kCameraBrightnessID, kCameraWhiteBalanceID

# those should appear in vision_definitions.py at some point
kTopCamera = 0
kBottomCamera = 1

class NaoCam (NaoNode):
    def __init__(self):
        NaoNode.__init__(self)
        rospy.init_node('nao_camera')

        self.camProxy = self.getProxy("ALVideoDevice")
        if self.camProxy is None:
            exit(1)
        self.nameId = ''
        self.camera_infos = {}
        def returnNone():
            return None
        self.config = defaultdict(returnNone)

        # ROS publishers
        self.pub_img_ = rospy.Publisher('~image_raw', Image)
        self.pub_info_ = rospy.Publisher('~camera_info', CameraInfo)

        # initialize the parameter server
        self.srv = Server(NaoCameraConfig, self.reconfigure)

    def reconfigure( self, new_config, level ):
        """ Reconfigure the camera
        """
        # check if we are even subscribed to a camera
        if new_config['source'] and not self.config['source']:
            # unsubscribe for all zombie subscribers
            self.camProxy.unsubscribeAllInstances("rospy_gvm")
            # subscribe
            self.nameId = self.camProxy.subscribe("rospy_gvm", new_config['resolution'], new_config['color_space'],
                                                  new_config['frame_rate'])
            rospy.loginfo('subscriber name is ' + self.nameId)

        if self.config['source'] != new_config['source']:
            rospy.loginfo('using camera: %d', new_config['source'])
            if new_config['source'] == kTopCamera:
                self.frame_id = "/CameraTop_frame"
            elif new_config['source'] == kBottomCamera:
                self.frame_id = "/CameraBottom_frame"
            else:
                rospy.logerr('Invalid source. Must be 0, 1 or 2')
                exit(1)

        # check if the camera changed
        if self.config['camera_info_url'] != new_config['camera_info_url'] and \
                        new_config['camera_info_url'] and new_config['camera_info_url'] not in self.camera_infos:
            if 'cim' not in self.__dict__:
                self.cim = camera_info_manager.CameraInfoManager(cname='nao_camera')

            if not self.cim.setURL( new_config['camera_info_url'] ):
                rospy.logerr('malformed URL for calibration file')
            else:
                try:
                    self.cim.loadCameraInfo()
                except IOExcept:
                    rospy.logerr('Could not read from existing calibration file')

            if self.cim.isCalibrated():
                rospy.loginfo('Successfully loaded camera info')
                self.camera_infos[new_config['camera_info_url']] = self.cim.getCameraInfo()
            else:
                rospy.logerr('There was a problem loading the calibration file. Check the URL!')

        # set params
        for key, naoqi_key in [('source', kCameraSelectID), ('auto_exposition', kCameraAutoExpositionID),
                               ('auto_exposure_algo', kCameraAecAlgorithmID),
                               ('contrast', kCameraContrastID), ('saturation', kCameraSaturationID),
                               ('hue', kCameraHueID), ('sharpness', kCameraSharpnessID),
                               ('auto_white_balance', kCameraAutoWhiteBalanceID)
                               ]:
            if self.config[key] != new_config[key]:
                self.camProxy.setParam(naoqi_key, new_config[key])

        for key, naoqi_key, auto_exp_val in [('exposure', kCameraExposureID, 0),
                                             ('gain', kCameraGainID, 0), ('brightness', kCameraBrightnessID, 1)]:
            if self.config[key] != new_config[key]:
                self.camProxy.setParam(kCameraAutoExpositionID, auto_exp_val)
                self.camProxy.setParam(naoqi_key, new_config[key])

        if self.config['white_balance'] != new_config['white_balance']:
            self.camProxy.setParam(kCameraAutoWhiteBalanceID, 0)
            self.camProxy.setParam(kCameraWhiteBalanceID, new_config['white_balance'])

        for key, method in [('resolution', 'setResolution'), ('color_space', 'setColorSpace'),
                            ('frame_rate', 'setFrameRate')]:
            if self.config[key] != new_config[key]:
                self.camProxy.__getattribute__(method)(self.nameId, new_config[key])

        self.config.update(new_config)

        return self.config

    def main_loop(self):
        img = Image()
        r = rospy.Rate(self.config['frame_rate'])
        while not rospy.is_shutdown():
            image = self.camProxy.getImageRemote(self.nameId)
            if image is None:
                continue
            # Deal with the image
            if self.config['use_ros_time']:
                img.header.stamp = rospy.Time.now()
            else:
                img.header.stamp = rospy.Time(image[4] + image[5]*1e-6)
            img.header.frame_id = self.frame_id
            img.height = image[1]
            img.width = image[0]
            nbLayers = image[2]
            if image[3] == kYUVColorSpace:
                encoding = "mono8"
            elif image[3] == kRGBColorSpace:
                encoding = "rgb8"
            elif image[3] == kBGRColorSpace:
                encoding = "bgr8"
            elif image[3] == kYUV422ColorSpace:
                encoding = "yuv422" # this works only in ROS groovy and later
            else:
                rospy.logerr("Received unknown encoding: {0}".format(image[3]))

            img.encoding = encoding
            img.step = img.width * nbLayers
            img.data = image[6]
            if self.config['camera_info_url'] in self.camera_infos:
                infomsg = self.camera_infos[self.config['camera_info_url']]
                infomsg.header = img.header
                self.pub_info_.publish(infomsg)

            self.pub_img_.publish(img)
            r.sleep()


        self.camProxy.unsubscribe(self.nameId)

if __name__ == "__main__":
    try:
        naocam = NaoCam()
        naocam.main_loop()
    except RuntimeError as e:
        rospy.logerr('Something went wrong: %s' % str(e) )
    rospy.loginfo('Camera stopped')
