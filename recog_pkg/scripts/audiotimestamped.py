#!/usr/bin/env python
from __future__ import division
import rospy
from recog_pkg.msg import AudioDataTimeStamped
from audio_common_msgs.msg import AudioData

class AudioTimeStamped():
    def __init__(self):
        rospy.Rate(100)
        self.pub = rospy.Publisher("~audiotime", AudioDataTimeStamped, queue_size=2**24)
        self.subscribe()

    def subscribe(self):
        self.sub = rospy.Subscriber("~audio", AudioData, self.cb, queue_size=1000, buff_size=2**24)

    def cb(self, msg):
        # self.data = msg.data
        # self.publish()
        ad = AudioDataTimeStamped()
        ad.data = msg.data
        ad.header.stamp = rospy.Time.now()
        self.pub.publish(ad)

    # def publish(self):
    #     msg = AudioDataTimeStamped(data=self.data)
    #     msg.header = rospy.Timenow()
    #     self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("audiotimestamped")
    AudioTimeStamped()
    rospy.spin()
