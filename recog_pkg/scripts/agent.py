#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import exceptions
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
#from recog_pkg.msg import Camera3d
import tf2_ros
import tf_conversions
import math
import numpy as np
from std_msgs.msg import Float32, Int32
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from jsk_recognition_msgs.msg import LabelArray, RectArray
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import cv_bridge
from recog_pkg.msg import Camera3d
import copy
import tmc_eus_py
from tmc_eus_py.coordinates import Coordinates


class Agent():
    #example go_to_microwave, base_pose, microwave_solve_ik 
    def __init__(self):
        print("a")
        self.robot = hsrb_interface.Robot()
        print("b")
        self.omni_base = self.robot.get('omni_base')
        print("c")
        self.whole_body = self.robot.get('whole_body')
        print("d")
        self.gripper = self.robot.get('gripper')
        print("e")

        #self.freq_list = []
        #self.freq_list_replace = []
        self.stop_yakan_tweak_flag = False
        self.u = 0
        self.v = 0
        self.switch = 0

        self.u_high = 250
        self.u_low = 240
        self.v_high = 260
        self.v_low = 250
        self.hand_done = False


    def kitchen_collision(self):
        collision_world.add_box(x=0.5, y=1.0, z=0.8, pose=geometry.pose(x=3.8, y=0.0, z=0.4), frame_id="map")
        #collision_world.remove_all()
        self.whole_body.collision_world = collision_world

    def go_to_kitchen(self):
        kitchen_pose = geometry.Pose(pos=geometry.Vector3(x=3.11, y=1.21, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=-0.01023836327822357, w=0.9999475865851083))
        self.omni_base.go_pose(kitchen_pose)

    def go_to_yakan(self):
        yakan_pose = geometry.Pose(pos=geometry.Vector3(x=3.108781824048564, y=0.037752328711472494, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=0.013848295774654074, w=0.9999041077544075))
        self.omni_base.go_pose(yakan_pose)

    def go_to_stop_button(self):
        stop_button_pose = geometry.Pose(pos=geometry.Vector3(x=3.1237696626280242, y=-0.31407748223644616, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=-0.01521901673895873, w=0.9998841840580838))
        self.omni_base.go_pose(stop_button_pose)
        
    def go_to_microwave(self):
        microwave_pose = geometry.Pose(pos=geometry.Vector3(x=1.909, y=1.779478, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=0.70096336, w=0.713197))
        self.omni_base.go_pose(microwave_pose)

    def go_to_fridge(self):
        fridge_pose = geometry.Pose(pos=geometry.Vector3(x=0.1457817782465553, y=1.5697370723246589, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=0.7400625754780703, w=0.6725380170494195))
        self.omni_base.go_pose(fridge_pose)

    def base_pose(self, x=0.2):
        self.whole_body.move_to_neutral()
        self.whole_body.move_to_joint_positions({"arm_lift_joint": x, "head_tilt_joint": 0})

    def lookup_transform(self, parent, child, stamp=rospy.Time(0)):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        t = tfBuffer.lookup_transform(parent, child, stamp)
        return t

    def get_bounding_box(self, color):
        base_to_target_point = None
        target_coords = None
        target_point = None
        msg = one_shot_subscribe("/HSI_color_filter/boxes_"+color, timeout=10000)
        print(msg.boxes)
        if msg:
            distance_x = 0
            min_x = 100000
            base_to_camera = None
            if base_to_camera == None:
                base_to_camera = self.lookup_transform("base_link", msg.boxes[0].header.frame_id)
            for i in range(len(msg.boxes)):
                #camera_to_target_coords = Coordinates(pos=msg.boxes[i].pose.position)
                base_to_target_point = msg.boxes[i].pose.position
                distance_x = base_to_target_point.x
                if min_x > distance_x:
                    min_x = distance_x
                    target_point = base_to_target_point
            return target_point
        else:
            print("msg is empty")

    def solve_ik(self, target, grasp_type, fl_vec=[0,0,0], frame="base_link"):
        self.whole_body.linear_weight = 100.0
        self.whole_body.angular_weight = 100.0
        if grasp_type == "front_horizontal":
            self.whole_body.move_end_effector_pose(
                geometry.pose(x=(target.x + fl_vec[0]),
                              y=(target.y + fl_vec[1]),
                              z=(target.z + fl_vec[2]),
                              ei=-math.pi/2, ek=-math.pi/2),
                ref_frame_id=frame)
            
        elif grasp_type == "front_vertical":
            self.whole_body.move_end_effector_pose(
                geometry.pose(x=(target.x + fl_vec[0]),
                              y=(target.y + fl_vec[1]),
                              z=(target.z + fl_vec[2]),
                              ej=math.pi/2),
                ref_frame_id=frame)
            
        elif grasp_type == "top":
            self.whole_body.move_end_effector_pose(
                geometry.pose(x=(target.x + fl_vec[0]),
                              y=(target.y + fl_vec[1]),
                              z=(target.z + fl_vec[2]),
                              ei=-math.pi*5/6, ek=-math.pi/2),
                ref_frame_id=frame)

    def microwave_solve_ik(self):
        #self.base_pose(x=0.3)
        microwave_point = self.get_bounding_box("red")
        self.solve_ik(microwave_point, "front_horizontal", fl_vec=[-0.2,0.0,0.0])
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

    def find_stop_button_pose(self):
        self.whole_body.move_to_neutral()
        self.whole_body.move_to_joint_positions({"arm_lift_joint" : 0.25})
        self.whole_body.move_to_joint_positions({"head_tilt_joint": -0.5})

    def stop_yakan_solve_ik(self):
        self.whole_body.linear_weight = 100.0
        self.whole_body.angular_weight = 100.0
        #self.gripper.apply_force(1.0)
        self.whole_body.move_end_effector_pose(
            geometry.pose(z=0.1, ei=math.pi), ref_frame_id="button2")
        self.stop_yakan_tweak_flag = True
        # self.whole_body.impedance_config = "compliance_hard"
        # self.whole_body.move_end_effector_pose(geometry.pose(z=0.06), ref_frame_id="hand_palm_link")
        # while not self.done:
        #     self.whole_body.move_end_effector_pose(geometry.pose(x= np.random.rand()/50, y=np.random.rand()/50, z=0.05), ref_frame_id="hand_palm_link")

        #     if self.freq_list_replace[-2:] == [0,4]:
        #         break
        # self.whole_body.impedance_config = None

    def stop_yakan_solve_ik2(self):
        self.whole_body.impedance_config = "compliance_soft"
        self.gripper.apply_force(1.0)
        #if 4 not in self.freq_list_replace:
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.10), ref_frame_id="hand_palm_link")
        #else:
        #    print("cc")

        
    def yakan_solve_ik(self):
        yakan_point = self.get_bounding_box("yellow")
        self.solve_ik(yakan_point, "front_horizontal", fl_vec=[-0.1, 0.0, 0.2])
        self.whole_body.move_end_effector_pose(
            geometry.pose(y=0.02), ref_frame_id="hand_palm_link")
        self.whole_body.move_end_effector_pose(
            geometry.pose(z=0.1), ref_frame_id="hand_palm_link")
        self.whole_body.move_end_effector_pose(
            geometry.pose(x=-0.02), ref_frame_id="hand_palm_link")
        
        self.gripper.apply_force(1.7)
        self.whole_body.move_end_effector_pose(geometry.pose(y=-0.05) ,ref_frame_id="hand_palm_link")

    # def get_frequency(self):
    #     self.flag = 0
    #     self.done = False
    #     #rospy.init_node("audio")
    #     rospy.Subscriber("/fft/max_freq_output", Float32, self.frequency_cb, queue_size=1)

    # def frequency_cb(self, msg1):
    #     frequency = msg1.data
    #     # self.freq_list.append(frequency)

    #     # if len(self.freq_list) > 30:
    #     #     self.freq_list.pop(0)

    #     # print("----------", len(self.freq_list))

    #     # for i in self.freq_list:
    #     #     if 1990 < i and i < 2010:
    #     #         self.freq_list_replace.append(1)
    #     #     elif 2490 < i and i < 2510:
    #     #         self.freq_list_replace.append(2)
    #     #     elif 2200 < i and i < 2300:
    #     #         self.freq_list_replace.append(3)
    #     #         self.flag = 3
    #     #     elif 1950< i and i < 1970:
    #     #         self.freq_list_replace.append(4)


    #     if 1990 < frequency and frequency < 2010:
    #         self.freq_list_replace.append(1)
    #         if len(self.freq_list_replace) > 30:
    #             self.freq_list_replace.pop(0)
    #     elif 2490 < frequency and frequency < 2510:
    #         self.freq_list_replace.append(2)
    #         if len(self.freq_list_replace) > 30:
    #             self.freq_list_replace.pop(0)
    #     elif 2200 < frequency and frequency < 2300:
    #         self.freq_list_replace.append(3)
    #         self.flag = 3
    #         if len(self.freq_list_replace) > 30:
    #             self.freq_list_replace.pop(0)
    #     elif 1957< frequency and frequency < 1962:
    #         self.freq_list_replace.append(4)
    #         if len(self.freq_list_replace) > 30:
    #             self.freq_list_replace.pop(0)


    #         #else:
    #         #    self.freq_list_replace.append(0)

    #     # while not self.done:
    #     #     if self.freq_list_replace[-2:] == [0,1]:
    #     #         print("aaa")
    #     #         #msg2 = one_shot_subscribe("/sound_direction")
    #     #         #print(self.msg2.data)
    #     #         #self.omni_base.go_rel(yaw= -self.msg2.data / (2*math.pi))
    #     #         #self.go_to_microwave()
    #     #         #self.base_pose()
    #     #         #rospy.sleep(2)
    #     #         #self.microwave_solve_ik()
    #     #         self.done = True
    #     #     elif self.freq_list_replace[-2:] == [0,2]:
    #     #         print("bbb")
    #     #         self.done = True
    #     #     elif self.freq_list_replace[-2:] == [0,3]:
    #     #         print("ccc")
    #     #         self.flag = 3
    #     #         self.done = True
    #     #     elif self.freq_list_replace[-2:] == [0,4]:
    #     #         print("ddd")
    #     #         self.done = True
    #     #     else:
    #     #         break
    #     print(self.freq_list_replace)
    
    # def run2(self):
    #     print(self.u)
    #     print(self.v)
    #     # if self.stop_yakan_tweak_flag:
    #     r = rospy.Rate(0.33)
    #     while not rospy.is_shutdown():
    #         print(self.freq_list_replace)

    #         if not self.done:
    #             if self.flag == 3:
    #                 #self.go_to_stop_button()
    #                 #self.find_stop_button_pose()
                    
    #                 self.stop_yakan_solve_ik()
    #                 self.flag = 0
    #                 self.done = True

    #         if self.done:
    #             if 4 in self.freq_list_replace:
    #                 break
                
    #             if self.switch == 0:
    #                 if self.u > self.u_high:
    #                     #self.whole_body.move_end_effector_pose(geometry.pose(y=-0.02), ref_frame_id="hand_palm_link")
    #                     self.omni_base.go_rel(y=-0.017)
    #                 elif self.u < self.u_low:
    #                     #self.whole_body.move_end_effector_pose(geometry.pose(y=0.02), ref_frame_id="hand_palm_link")
    #                     self.omni_base.go_rel(y=0.017)

    #                 if self.v > self.v_high:
    #                     #self.whole_body.move_end_effector_pose(geometry.pose(x=-0.02), ref_frame_id="hand_palm_link")
    #                     self.omni_base.go_rel(x=-0.017)
    #                 elif self.v < self.v_low:
    #                     #self.whole_body.move_end_effector_pose(geometry.pose(x=0.02), ref_frame_id="hand_palm_link")
    #                     self.omni_base.go_rel(x=0.017)
            
    #                 if ((self.u >= self.u_low and self.u <= self.u_high) and (self.v >= self.v_low and self.v <= self.v_high)):
    #                     self.switch = 1
    #                 else:
    #                     pass

    #             elif self.switch == 1:
    #                 self.whole_body.impedance_config = "compliance_hard"
    #                 if 4 not in self.freq_list_replace:
    #                     print("aaaaa")
    #                     self.gripper.apply_force(1.0)
    #                     self.whole_body.move_end_effector_by_line((0,0,1), 0.06)
    #                     self.gripper.command(1.0)
    #                 self.switch = 0
    #         r.sleep()

    # def get_3d_point_hand(self):
    #     #rospy.init_node("two2three_hand")
    #     #self.pub_viz = rospy.Publisher("/hand_camera_remote/image_raw", Image, queue_size=1)
    #     self.bridge = CvBridge()
    #     self.sub = message_filters.Subscriber("/hand_camera_remote/image_raw", Image, queue_size=1)
    #     #self.sub_labelArray = message_filters.Subscriber("/qatm/output/labels", LabelArray, queue_size=1)
    #     self.sub_rectArray = message_filters.Subscriber("/qatm2/output/rects", RectArray, queue_size=1)
    #     queue_size = rospy.get_param("~queue_size", 100)
    #     self.subs = [self.sub, self.sub_rectArray]
    #     slop = rospy.get_param("~slop", 0.1)
    #     self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size=queue_size, slop=slop)
    #     self.sync.registerCallback(self.hand_camera_cb)

    # def unsubscribe(self):
    #     for sub in self.subs:
    #         sub.unregister()

    # def hand_camera_cb(self, msg1, rectmsg):
    #     try:
    #         depth_image = self.bridge.imgmsg_to_cv2(msg1, "passthrough")
    #     except CvBridgeError, e:
    #         rospy.logerr(e)

    #     self.h, self.w, _ = depth_image.shape

    #     rect_info = rectmsg.rects[0]
    #     self.u = rect_info.x + rect_info.width/2
    #     self.v = rect_info.y + rect_info.height/2
    #     print(self.u, self.v)
    #     #output_image = self.plot_ok(depth_image)
    #     #msg_viz = cv_bridge.CvBridge().cv2_to_imgmsg(output_image, encoding="bgr8")
    #     #self.pub_viz.publish(msg_viz)

    # def plot_ok(self,image_raw, show=False, save_name=None):
    #     d_img = image_raw.copy()
    #     d_img = cv2.putText(d_img, "OK", (self.u_low, self.v_low), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0))
    #     d_img = cv2.rectangle(d_img, (self.u_low, self.v_low), (self.u_high, self.v_high), (255,0,0))
    #     if show:
    #         plt.imshow(d_img)
    #     if save_name:
    #         cv2.imwrite(save_name, d_img[:,:,::-1])
    #     return d_img

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
