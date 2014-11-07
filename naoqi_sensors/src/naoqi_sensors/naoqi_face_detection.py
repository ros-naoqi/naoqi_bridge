#!/usr/bin/env python

#import ROS dependencies
import rospy
from naoqi_msgs.msg import FaceDetection, FaceInfo

#import NAO dependencies
from naoqi_driver.naoqi_node import NaoqiNode


class NaoqiFaceDetection(NaoqiNode):
    NAOQI_FACE_DETECTION_SUB_NAME = 'ros_face_detection_subscription'
    def __init__(self, param_face_rate="~face_detection_rate", face_rate=10):
        NaoqiNode.__init__(self, "face_detection_publisher")
        self.faceRate = rospy.Rate(rospy.get_param(param_face_rate, face_rate))
        if rospy.has_param('~use_ros_time'):
            self.use_ros_time_ = rospy.get_param('~use_ros_time')
        else:
            self.use_ros_time_ = False

        self.pub_face_ = rospy.Publisher('~face_detection', FaceDetection)
        self.connectNaoQi()

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.faceProxy = self.get_proxy("ALFaceDetection")
        self.memProxy = self.get_proxy("ALMemory")
        if self.faceProxy is None or self.memProxy is None:
            exit(1)

    # do it!
    def run(self):
        # start subscriber to sonar sensor
        self.faceProxy.subscribe(self.NAOQI_FACE_DETECTION_SUB_NAME)

        while self.is_looping():
            val = self.memProxy.getData("FaceDetected", 0)

            # Check whether we got a valid output: a list with two fields.
            if(val and isinstance(val, list) and len(val) == 5):
                # We detected faces !
                # For each face, we can read its shape info and ID.
                # First Field = TimeStamp.
                timeStamp = val[0]
                # Second Field = array of face_Info's.
                faceInfoArray = val[1]

                face_detection_msg = FaceDetection()
                # camera_id
                face_detection_msg.camera_id = val[4]
                # timestamp
                if self.use_ros_time_:
                    face_detection_msg.header.stamp = rospy.Time.now()
                else:
                    face_detection_msg.header.stamp = rospy.Time(timeStamp[0], timeStamp[1])
                try:
                    # Browse the faceInfoArray to get info on each detected face.
                    for faceInfo in faceInfoArray:
                        if len(faceInfo) == 2:
                            faceShapeInfo = faceInfo[0]
                            faceExtraInfo = faceInfo[1]

                            face_info_msg = FaceInfo()
                            # shape info
                            face_info_msg.alpha = faceShapeInfo[1]
                            face_info_msg.beta = faceShapeInfo[2]
                            face_info_msg.width = faceShapeInfo[3]
                            face_info_msg.height = faceShapeInfo[4]
                            # extra info
                            face_info_msg.id = faceExtraInfo[0]
                            face_info_msg.score = faceExtraInfo[1]
                            face_info_msg.name = faceExtraInfo[2]
                            face_detection_msg.face_infos.append(face_info_msg)

                            print "  alpha %.3f - beta %.3f" % (faceShapeInfo[1], faceShapeInfo[2])
                            print "  width %.3f - height %.3f" % (faceShapeInfo[3], faceShapeInfo[4])
                except Exception, e:
                    print "faces detected, but it seems getData is invalid. ALValue ="
                    print val
                    print "Error msg %s" % (str(e))
                self.pub_face_.publish(face_detection_msg)
            else:
                print "Error with getData. ALValue = %s" % (str(val))

            self.faceRate.sleep()

        #exit face detection subscription
        self.faceProxy.unsubscribe(self.NAOQI_FACE_DETECTION_SUB_NAME)

