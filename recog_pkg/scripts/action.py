#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: lfurushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import hsrb_interface
import traceback
import tf2_geometry_msgs


class ComplianceHardWithDump(object):
    '''
    Temporarily change rosparam
    `/hsrb/impedance_control/compliance_hard/impedance/translation/[xyz]/[mdk]`
    to realize hard impedance dumping with only specifed axes.
    As a setting of dumping, 
    `/hsrb/impedance_control/dumper_soft/impedance/translation/[xyz]/[mdk]`
    is used.
    
    Usage:
    with compliance_hard_with_dump(whole_body, 'xy'):
        whole_body.move_end_effector_pose(...)
    '''

    coeffs = 'mdk'

    def __init__(self, whole_body, axes):
        self.whole_body = whole_body
        self.axes = ''
        if 'x' in axes:
            self.axes += 'x'
        if 'y' in axes:
            self.axes += 'y'
        if 'z' in axes:
            self.axes += 'z'
        self.original_params = {}
        self.original_config = ''

    def __enter__(self):
        for axis in self.axes:
            self.original_params[axis] = {}
            for coeff in self.coeffs:
                target_param = '/hsrb/impedance_control/compliance_hard/impedance/translation/' + axis + '/' + coeff
                ref_param = '/hsrb/impedance_control/dumper_soft/impedance/translation/' + axis + '/' + coeff
                
                self.original_params[axis][coeff] \
                    = rospy.get_param(target_param)
                rospy.set_param(
                    target_param, rospy.get_param(ref_param))

        self.original_config = self.whole_body.impedance_config
        self.whole_body.impedance_config = 'compliance_hard'

        return self

    def __exit__(self, ex_type, ex_value, trace):
        for axis in self.axes:
            for coeff in self.coeffs:
                target_param = '/hsrb/impedance_control/compliance_hard/impedance/translation/' + axis + '/' + coeff
                rospy.set_param(
                    target_param, self.original_params[axis][coeff])

        self.whole_body.impedance_config = self.original_config

class Action(object):
    ###
    ### TODO ###
    ### initial_pose
    ###
    grasp_pose = {
        'arm_flex_joint': -0.0026787544868800417,
        'arm_lift_joint': 0.2999983031559333,
        'arm_roll_joint': 1.5699988638605271,
        'head_pan_joint': -0.07757885596160996,
        'head_tilt_joint': -0.40000182600196055,
        'wrist_flex_joint': -1.5347050039870336,
        'wrist_roll_joint': 4.1702398501097804e-05
    }
    
    map_frame = "map"
    base_frame = "base_link"
    sensor_frame = "head_rgbd_sensor_rgb_frame"
    end_effector_frame = "hand_palm_link"
    
    robot = None
    whole_body = None
    gripper = None
    omni_base = None
    collision_world = None
    wrist_wrench = None
    tfl = None

    prev_action = None
    next_action = None
    recover_action = None
    name = ""

    @classmethod
    def set_robot(cls, robot=None):
        if robot:
            cls.robot = robot
        else:
            cls.robot = hsrb_interface.Robot()
        cls.whole_body = cls.robot.get("whole_body")
        cls.omni_base = cls.robot.get("omni_base")
        cls.gripper = cls.robot.get("gripper")
        cls.wrist_wrench = cls.robot.try_get("wrist_wrench")
        cls.collision_world = cls.robot.get("global_collision_world")
        cls.tts = cls.robot.get("default_tts")
        cls.tts.language = cls.tts.ENGLISH
        #cls.tts.language = cls.tts.JAPANESE
        cls.tfl = cls.robot._get_tf2_buffer()

    def __init__(self):
        assert len(self.name) > 0

    def run(self, **args):
        raise NotImplementedError()

    def __call__(self, **args):
        if Action.robot is None:
            Action.set_robot()
        return self.run(**args)

    def then(self, act):
        act.prev_action = self
        self.next_action = act
        return act

    def recover(self, act):
        act.prev_action = self
        self.recover_action = act
        act.next_action = self
        return self

    def execute(self, **args):
        if Action.robot is None:
            Action.set_robot()

        act = self
        while act.prev_action is not None:
            act = act.prev_action

        print "Task start with %s" % act.name

        while True:
            if rospy.is_shutdown():
                return
            if act is None:
                rospy.loginfo("task finished with success: %s" % success)
                return success
            try:
                if act.prev_action:
                    rospy.loginfo("%s -> %s (%s)" % (act.prev_action.name, act.name, success))
                rospy.logdebug("running %s with arguments: %s" % (act.name, args))
                success, rets = act(**args)
                rospy.logdebug("finished %s with %s: %s" % (act.name, success, rets))
            except Exception as e:
                rospy.logerr(e)
                rospy.logerr(traceback.format_exc())
                success, rets = False, {}
            args.update(rets)
            if success:
                act = act.next_action
            else:
                act = act.recover_action


if __name__ == '__main__':
    Action()()
