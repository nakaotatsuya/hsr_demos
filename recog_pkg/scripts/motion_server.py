#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse

#rospy.init_node("service")
#rospy.Service('hoge', Trigger, callback)
#rospy.spin()

def callback(req):
    # hsrb
    return EmptyResponse(success = True,
                         message = "aaa")
