#!/usr/bin/env python
import roslib
roslib.load_manifest("recog_pkg")
import rospy
from recog_pkg.msg import Camera3d
import tf
#import numpy as np

class TfBroadCaster():
    def __init__(self):
        self.msg_x = []
        self.msg_y = []
        self.msg_z = [0]*20
        self.stop_button = rospy.get_param("~button")
        rospy.Subscriber("~input", Camera3d, self.button_pose, self.stop_button)

    def button_pose(self, msg, stop_button):
        br = tf.TransformBroadcaster()
        #msg_x.append(msg.x)
        #msg_y.append(msg.y)
        self.msg_z.append(msg.z)
        del self.msg_z[0]
        #print(msg_z)
        z = sum(self.msg_z) / 20
        br.sendTransform((msg.x, msg.y, z),
                         (0,0,0,1),
                         rospy.Time.now(),
                         stop_button,
                         "head_rgbd_sensor_rgb_frame")

if __name__ == "__main__":
    rospy.init_node("tf_broadcaster")
    tfbroadcaster = TfBroadCaster()
    #stop_button = rospy.get_param("~button")
    #rospy.Subscriber("~input", Camera3d, button_pose, stop_button)
    rospy.spin()
