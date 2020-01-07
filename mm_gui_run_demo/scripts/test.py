#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PointStamped

class UR5MoveGroup():
	def __init__(self):
		rospy.init_node('mm_moveit_gui_execution', anonymous=True)

		# moveit commander
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		self.group = moveit_commander.MoveGroupCommander("ur5")  		
		self.group.set_planner_id("RRTConnectkConfigDefault")
		self.group.set_planning_time(5.0)
		self.group.set_num_planning_attempts(10)
		self.group.set_max_velocity_scaling_factor(0.1)
		self.group.set_max_acceleration_scaling_factor(0.1)