#!/usr/bin/env python
import roslib
roslib.load_manifest("recog_pkg")
import rospy
from recog_pkg.msg import Camera3d
import tf

def button_pose(msg, stop_button):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, msg.z),
                     (0,0,0,1),
                     rospy.Time.now(),
                     stop_button,
                     "head_rgbd_sensor_rgb_frame")

if __name__ == "__main__":
    rospy.init_node("tf_broadcaster")
    stop_button = rospy.get_param("~button")
    rospy.Subscriber("~input", Camera3d, button_pose, stop_button)
    rospy.spin()
