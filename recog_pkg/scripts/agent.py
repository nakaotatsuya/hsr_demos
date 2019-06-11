#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import exceptions
from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf_conversions
import math


class Agent():
    #example go_to_microwave, base_pose, microwave_solve_ik 
    def __init__(self):
        self.robot = hsrb_interface.Robot()
        self.omni_base = self.robot.get('omni_base')
        self.whole_body = self.robot.get('whole_body')
        self.gripper = self.robot.get('gripper')

    def go_to_kitchen(self):
        kitchen_pose = geometry.Pose(pos=geometry.Vector3(x=3.11, y=1.21, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=-0.01023836327822357, w=0.9999475865851083))
        self.omni_base.go_pose(kitchen_pose)

    def go_to_microwave(self):
        microwave_pose = geometry.Pose(pos=geometry.Vector3(x=1.909, y=1.779478, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=0.70096336, w=0.713197))
        self.omni_base.go_pose(microwave_pose)

    def go_to_fridge(self):
        fridge_pose = geometry.Pose(pos=geometry.Vector3(x=0.1457817782465553, y=1.5697370723246589, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=0.7400625754780703, w=0.6725380170494195))
        self.omni_base.go_pose(fridge_pose)

    def base_pose(self):
        self.whole_body.move_to_neutral()
        self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.3, "head_tilt_joint": 0})

    def lookup_transform(self, parent, child):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        t = tfBuffer.lookup_transform(parent, child, rospy.Time(0))
        return t

    def get_bounding_box(self):
        base_to_target_point = None
        self.target_coords = None
        msg = one_shot_subscribe("/HSI_color_filter/boxes_red", timeout=10000)
        print(msg.boxes)
        if msg:
            distance_x = 0
            min_x = 100000
            #base_to_camera = None
            #if base_to_camera == None:
            #    base_to_camera = self.lookup_transform("base_link", msg.boxes[0].header.frame_id)
            for i in range(len(msg.boxes)):
                base_to_target_point = msg.boxes[i].pose.position
                distance_x = base_to_target_point.x
                if min_x > distance_x:
                    min_x = distance_x
                    self.target_coords = base_to_target_point
            return self.target_coords
        else:
            print("msg is empty")

    def solve_ik(self, target, fl_vec=[0,0,0], frame="base_link"):
        self.whole_body.linear_weight = 100.0
        self.whole_body.angular_weight = 100.0
        self.whole_body.move_end_effector_pose(
            geometry.pose(x=(target.x + fl_vec[0]),
                          y=(target.y + fl_vec[1]),
                          z=(target.z + fl_vec[2]),
                          ei=-math.pi/2, ek=-math.pi/2),
                          ref_frame_id=frame)

    def microwave_solve_ik(self):
        microwave_point = self.get_bounding_box()
        self.solve_ik(microwave_point, fl_vec=[-0.2,0.0,0.0])
        self.whole_body.move_end_effector_pose(
            geometry.pose(z=0.14), ref_frame_id="hand_palm_link")
        self.gripper.apply_force(2.3)
        #self.whole_body.linear_weight = 100.0
        #self.whole_body.impedance_config = "compliance_hard"
        
        while 1:
            try:
                self.whole_body.move_end_effector_by_arc(geometry.pose(y=0.32, z=0.07, ej=math.radians(90.0)), math.radians(30.0), ref_frame_id="hand_palm_link")
            except exceptions.MotionPlanningError:
                print("aa")
                pass
            else:
                break
        self.gripper.command(1.0)
        self.whole_body.move_end_effector_pose(geometry.pose(z=-0.2), ref_frame_id="hand_palm_link")
        while 1:
            try:
                self.whole_body.move_end_effector_by_arc(geometry.pose(ej = math.radians(90.0)), math.radians(-30.0), ref_frame_id="hand_palm_link")
            except exceptions.MotionPlanningError:
                print("bb")
                pass
            else:
                break
        self.whole_body.move_end_effector_pose(geometry.pose(y=-0.15), ref_frame_id="hand_palm_link")
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.15, ek=-math.pi/2), ref_frame_id="hand_palm_link")
        # self.whole_body.impedance_config = "compliance_soft"
        # self.whole_body.linear_weight = 30.0
        self.whole_body.move_end_effector_pose(geometry.pose(x=-0.15), ref_frame_id="hand_palm_link")
        # self.whole_body.impedance_config = None

    #def fridge_solve_ik(self):
        
#def callback(msg):
#    boundingBoxList = msg.boxes
#    print(boundingBoxList)
    
def one_shot_subscribe(topic_name, mclass=None,
                       timeout=None, after_stamp=None,
                       condition=None,
                       frequency=10):
    """
    Subscribe message, just for once
    """
    if mclass is None:
        import rostopic
        import importlib
        import rosgraph
        master = rosgraph.masterapi.Master('/rostopic')
        topic_types = dict(rostopic._master_get_topic_types(master))
        topic_type = topic_types[topic_name].split('/')
        pkg = importlib.import_module('{}.msg'.format(topic_type[0]))
        mclass = getattr(pkg, topic_type[1])

    if after_stamp is not None and isinstance(after_stamp, rospy.Time):
        raise TypeError('after_stamp should be rospy.Time, but get {}'
                        .format(type(after_stamp)))

    class OneShotSubscriber(object):

        def __init__(self):
            self.msg = None
            self.sub = rospy.Subscriber(
                topic_name, mclass,
                self.one_shot_subscribe)

        def unsubscribe(self):
            self.sub.unregister()

        def one_shot_subscribe(self, msg):

            if after_stamp is not None:
                if msg.header.stamp.to_sec() > after_stamp.to_sec():
                    self.msg = msg
            else:
                self.msg = msg

    oss = OneShotSubscriber()

    finish_time = None
    if timeout is not None:
        finish_time = rospy.Time.now()
    if finish_time is not None:
        finish_time = finish_time + rospy.Duration(timeout / 1000.0)

    r = rospy.Rate(frequency)
    while not rospy.is_shutdown() and \
        not (finish_time is not None and
             (finish_time - rospy.Time.now()).to_sec < 0.0):
        r.sleep()
        if oss.msg is not None:
            if (condition is not None):
                if callable(condition) and \
                   condition(oss.msg):
                    break
            else:
                break
    oss.unsubscribe()
    return oss.msg


# if __name__ == "__main__":
#     rospy.init_node("bounding_box_array2")
#     get_bounding_box()
#     #msg = rospy.Subscriber("/HSI_color_filter/boxes_red", BoundingBoxArray, callback)
#     #msg = one_shot_subscribe("/HSI_color_filter/boxes_red")
#     #print(msg.boxes)
#     rospy.spin()
