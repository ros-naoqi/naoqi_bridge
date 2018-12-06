#!/usr/bin/env python

'''
ALTextToSpeech/Status
(http://doc.aldebaran.com/2-5/naoqi/audio/altexttospeech-api.html#ALTextToSpeech/Status)
value: [idOfConcernedTask, status]
status: "enqueued", "started", "thrown", "stopped", "done"

[flow of status]
(enqueued) => started => (thrown/ stopped/ done)
      (speech_started_status)  (speech_finished_status)

[table of flow of two variables]
           self.speech_started_status |  self.speech_finished_status
--------------------------------------------------------------------
enqueued            -----                     -----
started            x -> o                    o -> x
thrown             o -> x                    x -> o
stopped            o -> x                    x -> o
done               o -> x                    x -> o
'''

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_msgs.msg import Bool

class NaoqiSpeech(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_speech')
        self.connectNaoQi()
        self.rate = rospy.Rate(1)
        self.speechStartedPub = rospy.Publisher("speech_started", Bool, queue_size=10)
        self.speechFinishedPub = rospy.Publisher("speech_finished", Bool, queue_size=10)
        self.speech_started_status = False
        self.speech_finished_status = True
   
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        if self.memProxy is None:
            rospy.logerr("Could not get a proxy to ALMemory")
            exit(1)

    def run(self):
        while self.is_looping():
            try:
                if self.speechStartedPub.get_num_connections() > 0:
                    test = self.memProxy.getData("ALTextToSpeech/Status")
                    if self.speech_started_status == False and self.speech_finished_status == True and test[1] == "started":
                        self.speech_started_status = True
                        self.speech_finished_status = False
                        tmp=Bool()
                        tmp.data=True
                        self.speechStartedPub.publish(tmp)

                if self.speechFinishedPub.get_num_connections() > 0:
                    test = self.memProxy.getData("ALTextToSpeech/Status")
                    if self.speech_started_status == True and self.speech_finished_status == False and (test[1] == "thrown" or test[1] == "stopped" or test[1] == "done"):
                        self.speech_finished_status = True
                        self.speech_started_status = False
                        tmp=Bool()
                        tmp.data=True
                        self.speechFinishedPub.publish(tmp)
            except RuntimeError, e:
                pass
            self.rate.sleep()

if __name__ == '__main__':
    speech = NaoqiSpeech()
    speech.start()
    rospy.spin()
