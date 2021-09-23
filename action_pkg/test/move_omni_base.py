#!/usr/bin/env python

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#import control_msgs.msg
#import controller_manager_msgs.srv
import rospy
#import trajectory_msgs.msg
import tf.transformations

rospy.init_node("test")

#initialize action client
cli = actionlib.SimpleActionClient(
    "/move_base/move",
    MoveBaseAction)

cli.wait_for_server()

# pub = rospy.Publisher("goal",
#                       PoseStamped, queue_size=10)

# while pub.get_num_connections() < 2:
#     rospy.sleep(0.1)

# rospy.wait_for_service("/hsrb/controller_manager/list_controllers")
# list_controllers = rospy.ServiceProxy("/hsrb/controller_manager/list_controllers",
#                                       controller_manager_msgs.srv.ListControllers)

# running = False
# while running is False:
#     rospy.sleep(0.1)
#     for c in list_controllers().controller:
#         if c.name == "arm_trajectory_controller" and c.state == "running":
#             running = True

goal_x = 0.3
goal_y = 0
goal_yaw = 1.57

pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "base_link"
pose.pose.position = Point(goal_x, goal_y, 0)
quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
pose.pose.orientation = Quaternion(*quat)

goal = MoveBaseGoal()
goal.target_pose = pose

cli.send_goal(goal)
cli.wait_for_result()

# pub.publish(goal)

action_state = cli.get_state()
print(action_state)
if action_state == GoalStatus.SUCCEEDED:
    rospy.loginfo("Navigation Succeeded.")
