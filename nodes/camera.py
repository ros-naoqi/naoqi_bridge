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
from distutils.version import LooseVersion
import rospy
from sensor_msgs.msg import Image, CameraInfo
from nao_driver.nao_driver_naoqi import NaoNode
import camera_info_manager

from dynamic_reconfigure.server import Server
from nao_sensors.cfg import NaoCameraConfig

# import resolutions
from nao_sensors.vision_definitions import k960p, k4VGA, kVGA, kQVGA, kQQVGA
# import color spaces
from nao_sensors.vision_definitions import kYUV422ColorSpace, kYUVColorSpace, \
                    kRGBColorSpace, kBGRColorSpace, kDepthColorSpace
# import extra parameters
from nao_sensors.vision_definitions import kCameraSelectID, kCameraAutoExpositionID, kCameraAecAlgorithmID, \
                  kCameraContrastID, kCameraSaturationID, kCameraHueID, kCameraSharpnessID, kCameraAutoWhiteBalanceID, \
                  kCameraExposureID, kCameraGainID, kCameraBrightnessID, kCameraWhiteBalanceID

# those should appear in vision_definitions.py at some point
kTopCamera = 0
kBottomCamera = 1
kDepthCamera = 2

class NaoCam (NaoNode):
    def __init__(self):
        NaoNode.__init__(self, 'nao_camera')

        self.camProxy = self.get_proxy("ALVideoDevice")
        if self.camProxy is None:
            exit(1)
        self.nameId = None
        self.camera_infos = {}
        def returnNone():
            return None
        self.config = defaultdict(returnNone)

        # ROS publishers
        self.pub_img_ = rospy.Publisher('~image_raw', Image, queue_size=5)
        self.pub_info_ = rospy.Publisher('~camera_info', CameraInfo, queue_size=5)

        # initialize the parameter server
        self.srv = Server(NaoCameraConfig, self.reconfigure)

        # initially load configurations
        self.reconfigure(self.config, 0)

    def reconfigure( self, new_config, level ):
        """
        Reconfigure the camera
        """
        rospy.loginfo('reconfigure changed')
        if self.pub_img_.get_num_connections() == 0:
            rospy.loginfo('Changes recorded but not applied as nobody is subscribed to the ROS topics.')
            self.config.update(new_config)
            return self.config

        # check if we are even subscribed to a camera
        is_camera_new = self.nameId is None

        if is_camera_new:
            rospy.loginfo('subscribed to camera proxy, since this is the first camera')
            if self.get_version() < LooseVersion('2.0'):
                self.nameId = self.camProxy.subscribe("rospy_gvm", new_config['source'],
                                                      new_config['resolution'], new_config['color_space'],
                                                      new_config['frame_rate'])
            else:
                self.nameId = self.camProxy.subscribeCamera("rospy_gvm", new_config['source'],
                                                            new_config['resolution'], new_config['color_space'],
                                                            new_config['frame_rate'])

        if self.config['source'] != new_config['source'] or is_camera_new:
            rospy.loginfo('updating camera source information')

            if new_config['source'] == kTopCamera:
                self.frame_id = "/CameraTop_frame"
            elif new_config['source'] == kBottomCamera:
                self.frame_id = "/CameraBottom_frame"
            elif new_config['source'] == kDepthCamera:
                self.frame_id = new_config['camera3d_frame']
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
        key_naoqi_keys = [('auto_exposition', kCameraAutoExpositionID),
                          ('auto_exposure_algo', kCameraAecAlgorithmID),
                          ('contrast', kCameraContrastID), ('saturation', kCameraSaturationID),
                          ('hue', kCameraHueID), ('sharpness', kCameraSharpnessID),
                          ('auto_white_balance', kCameraAutoWhiteBalanceID)]
        if self.get_version() < LooseVersion('2.0'):
            key_method.append(('source', 'setActiveCamera'))

        for key, naoqi_key in key_naoqi_keys:
            if self.config[key] != new_config[key] or is_camera_new:
                if self.get_version() < LooseVersion('2.0'):
                    self.camProxy.setParam(naoqi_key, new_config[key])
                else:
                    self.camProxy.setCamerasParameter(self.nameId, naoqi_key, new_config[key])

        for key, naoqi_key, auto_exp_val in [('exposure', kCameraExposureID, 0),
                                             ('gain', kCameraGainID, 0), ('brightness', kCameraBrightnessID, 1)]:
            if self.config[key] != new_config[key] or is_camera_new:
                if self.get_version() < LooseVersion('2.0'):
                    self.camProxy.setParam(kCameraAutoExpositionID, auto_exp_val)
                    self.camProxy.setParam(naoqi_key, new_config[key])
                else:
                    self.camProxy.setCamerasParameter(self.nameId, kCameraAutoExpositionID, auto_exp_val)
                    self.camProxy.setCamerasParameter(self.nameId, naoqi_key, new_config[key])

        if self.config['white_balance'] != new_config['white_balance'] or is_camera_new:
            if self.get_version() < LooseVersion('2.0'):
                self.camProxy.setParam(kCameraAutoWhiteBalanceID, 0)
                self.camProxy.setParam(kCameraWhiteBalanceID, new_config['white_balance'])
            else:
                self.camProxy.setCamerasParameter(self.nameId, kCameraAutoWhiteBalanceID, 0)
                self.camProxy.setCamerasParameter(self.nameId, kCameraWhiteBalanceID, new_config['white_balance'])

        key_methods =  [ ('resolution', 'setResolution'), ('color_space', 'setColorSpace'), ('frame_rate', 'setFrameRate')]
        if self.get_version() >= LooseVersion('2.0'):
            key_methods.append(('source', 'setActiveCamera'))
        for key, method in key_methods:
            if self.config[key] != new_config[key] or is_camera_new:
                self.camProxy.__getattribute__(method)(self.nameId, new_config[key])

        self.config.update(new_config)

        return self.config

    def run(self):
        img = Image()
        r = rospy.Rate(self.config['frame_rate'])
        while self.is_looping():
            if self.pub_img_.get_num_connections() == 0:
                if self.nameId:
                    rospy.loginfo('Unsubscribing from camera as nobody listens to the topics.')
                    self.camProxy.unsubscribe(self.nameId)
                    self.nameId = None
                r.sleep()
                continue
            if self.nameId is None:
                self.reconfigure(self.config, 0)
                r.sleep()
                continue
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
            elif image[3] == kDepthColorSpace:
                encoding = "mono16"
            else:
                rospy.logerr("Received unknown encoding: {0}".format(image[3]))
            img.encoding = encoding
            img.step = img.width * nbLayers
            img.data = image[6]

            self.pub_img_.publish(img)

            # deal with the camera info
            if self.config['source'] == kDepthCamera and image[3] == kDepthColorSpace:
                infomsg = CameraInfo()
                # yes, this is only for an XTion / Kinect but that's the only thing supported by NAO
                ratio_x = float(640)/float(img.width)
                ratio_y = float(480)/float(img.height)
                infomsg.width = img.width
                infomsg.height = img.height
                # [ 525., 0., 3.1950000000000000e+02, 0., 525., 2.3950000000000000e+02, 0., 0., 1. ]
                infomsg.K = [ 525, 0, 3.1950000000000000e+02,
                              0, 525, 2.3950000000000000e+02,
                              0, 0, 1 ]
                infomsg.P = [ 525, 0, 3.1950000000000000e+02, 0,
                              0, 525, 2.3950000000000000e+02, 0,
                              0, 0, 1, 0 ]
                for i in range(3):
                    infomsg.K[i] = infomsg.K[i] / ratio_x
                    infomsg.K[3+i] = infomsg.K[3+i] / ratio_y
                    infomsg.P[i] = infomsg.P[i] / ratio_x
                    infomsg.P[4+i] = infomsg.P[4+i] / ratio_y

                infomsg.D = []
                infomsg.binning_x = 0
                infomsg.binning_y = 0
                infomsg.distortion_model = ""

                infomsg.header = img.header
                self.pub_info_.publish(infomsg)
            elif self.config['camera_info_url'] in self.camera_infos:
                infomsg = self.camera_infos[self.config['camera_info_url']]

                infomsg.header = img.header
                self.pub_info_.publish(infomsg)

            r.sleep()

        if (self.nameId):
            rospy.loginfo("unsubscribing from camera ")
            self.camProxy.unsubscribe(self.nameId)

if __name__ == "__main__":
    naocam = NaoCam()
    naocam.start()
    rospy.spin()
