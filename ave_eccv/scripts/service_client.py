#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
import sys

def call_service(data):
    print("saa")
    #rospy.loginfo("waiting service")
    rospy.wait_for_service("save_shots")
    try:
        service = rospy.ServiceProxy("save_shots", SetBool)
        resp = service()
        #return resp.success, resp.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
if __name__=="__main__":
    call_service(data=True)
