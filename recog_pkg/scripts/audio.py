#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import exceptions
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
import math
import message_filters
from agent import Agent
#from agent import one_shot_subscribe

class Audio(Agent):
    #name = "audio"
    def get_frequency(self):
        self.freq_list = []
        self.done = False
        
        # self.sub_max_freq =  message_filters.Subscriber("/fft/max_freq_output", Float32, queue_size=1)
        # self.sub_direction = message_filters.Subscriber("/sound_direction", Int32, queue_size=1)
        # queue_size= rospy.get_param("~queue_size", 100)
        # if rospy.get_param("~approximate_sync", False):
        #     self.sync = message_filters.TimeSynchronizer(
        #         [self.sub_max_freq, self.sub_direction], queue_size=queue_size)
        # else:
        #     slop = rospy.get_param("~slop", 0.1)
        #     self.sync = message_filters.ApproximateTimeSynchronizer(
        #         [self.sub_max_freq, self.sub_direction], queue_size=queue_size, slop=slop)
        # self.sync.registerCallback(self._cb)
        rospy.Subscriber("/fft/max_freq_output", Float32, self._cb, queue_size=1)
        #rospy.Subscriber("/sound_direction", Int32, self._cb2, queue_size=1)


    def _cb(self, msg1):
        frequency = msg1.data
        self.freq_list.append(frequency)
        self.freq_list_replace = []
        for i in self.freq_list:
            if ((1990 < i) & (i < 2010)):
                self.freq_list_replace.append(1)
            elif ((2490 < i) &(i < 2510)):
                self.freq_list_replace.append(2)
            else:
                self.freq_list_replace.append(0)
        #self.freq_list_replace = [1 if ((1990 < i) & (i < 2010)) else 0 for i in self.freq_list]
        #self.freq_list_replace = [2 if ((2490 < i) & (i < 2510)) else 0 for i in self.freq_list]
        #print(len(self.freq_list))
        #print(self.freq_list)
        #print(self.freq_list_replace)
        while not self.done:
            if self.freq_list_replace[-2:] == [0,1]:
                print("aaa")
                #msg2 = one_shot_subscribe("/sound_direction")
                #print(self.msg2.data)
                #self.omni_base.go_rel(yaw= -self.msg2.data / (2*math.pi))
                #self.go_to_microwave()
                #self.base_pose()
                #rospy.sleep(2)
                #self.microwave_solve_ik()
                self.done = True
            elif self.freq_list_replace[-2:] == [0,2]:
                print("bbb")
                self.done = True
            elif self.freq_list_replace[-2:] == [0,3]:
                print("ccc")
                self.done = True
            else:
                break


    #def _cb2(self, msg2):
    #    self.msg2 = msg2
    #def get_microwave_direction(self):
    #    self.msg2 = rospy.Subscriber("/sound/direction", Int32, queue_size=1)
    #    print(self.msg2.data)

if __name__ == "__main__":
    rospy.init_node("audio")
    agent = Audio()
    agent.get_frequency()
    rospy.spin()