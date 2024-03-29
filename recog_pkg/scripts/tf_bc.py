#!/usr/bin/env python
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

if __name__ == "__main__":
    rospy.init_node("button2_bc")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    r = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        try:
            button1_t = tfBuffer.lookup_transform("base_link","button1", rospy.Time(0), rospy.Duration(2.0))
            #rospy.loginfo("aaa")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("LookupTransform Error")
            rospy.sleep(1.0)
            continue
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "button2"

        #print(button1_t)
        #rospy.loginfo("aaaaaaaaaa")
        translation_x = button1_t.transform.translation.x
        translation_y = button1_t.transform.translation.y
        translation_z = button1_t.transform.translation.z
        t.transform.translation.x = translation_x
        t.transform.translation.y = translation_y
        t.transform.translation.z = translation_z
        q = tf_conversions.transformations.quaternion_from_euler(0,0,0,"sxyz")
        rotation = Quaternion(*q)
        t.transform.rotation = rotation
        br.sendTransform(t)
        r.sleep()
