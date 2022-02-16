#!/usr/bin/env python

import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

rospy.init_node("test")

#initialize action client
cli = actionlib.SimpleActionClient(
    "/hsrb/arm_trajectory_controller/follow_joint_trajectory",
    control_msgs.msg.FollowJointTrajectoryAction)

cli.wait_for_server()

# pub = rospy.Publisher("/hsrb/arm_trajectory_controller/command",
#                       trajectory_msgs.msg.JointTrajectory, queue_size=10)

# while pub.get_num_connections() == 0:
#     rospy.sleep(0.1)

rospy.wait_for_service("/hsrb/controller_manager/list_controllers")
list_controllers = rospy.ServiceProxy("/hsrb/controller_manager/list_controllers",
                                      controller_manager_msgs.srv.ListControllers)

running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == "arm_trajectory_controller" and c.state == "running":
            running = True

goal = control_msgs.msg.FollowJointTrajectoryGoal()
traj = trajectory_msgs.msg.JointTrajectory()
traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [0.2, -0.5, 0, 0, 0]
p.velocities = [0, 0, 0, 0, 0]
p.time_from_start = rospy.Time(3)
traj.points = [p]
goal.trajectory = traj

cli.send_goal(goal)

rospy.sleep(1)

p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [0, 0, 0, 0, 0]
p.velocities = [0, 0, 0, 0, 0]
p.time_from_start = rospy.Time(3)
traj.points = [p]
goal.trajectory = traj

cli.send_goal(goal)

cli.wait_for_result()
#pub.publish(traj)
