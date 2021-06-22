#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
import sys
from sound_classification.msg import InSound

class ServiceClientMic():
    def __init__(self):
        # self.in_sound = rospy.get_param(
        #     "~in_sound",
        #     "/sound_detector_volume/in_sound")
        self.subscribe_in_sound()

    def subscribe_in_sound(self):
        self.sub_in_sound = rospy.Subscriber("~in_sound", InSound, self.callback_in_sound, queue_size=1)

    def unsubscribe(self):
        self.sub_in_sound.unregister()

    def callback_in_sound(self, msg):
        if msg.in_sound:
            print("a")
            self.call_service_mic(data=True)
        else:
            pass

    def call_service_mic(self, data):
        print("s")
        rospy.wait_for_service("save_wave")
        rospy.sleep(4.0)
        try:
        
            service = rospy.ServiceProxy("save_wave", SetBool)
            resp = service()
            #return resp.success, resp.message
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def run(self):
        try:
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            sys.exit(0)

if __name__ == "__main__":
    rospy.init_node("service_client_mic", anonymous=True)
    service_client_mic = ServiceClientMic()
    service_client_mic.run()
