#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarkerUpdate

class UR5MoveGroupGUI(object):
	def __init__(self):
		super(UR5MoveGroupGUI, self).__init__()

		# moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('mm_moveit_gui_execution', anonymous=True)

		# Subscriber
		rospy.Subscriber("rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", InteractiveMarkerUpdate, self.ee_pose_update_cb)
		rospy.Subscriber("ur5_plan", Bool, self.ur5_plan_cb)
		rospy.Subscriber("ur5_execute", Bool, self.ur5_execute_cb)  
		rospy.Subscriber("ur5_stop", Bool, self.ur5_stop_cb)

		# Publisher
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=50)

		# moveit commander
		self.robot = moveit_commander.RobotCommander()
  		self.scene = moveit_commander.PlanningSceneInterface()
  		self.group = moveit_commander.MoveGroupCommander("manipulator")  		
  		self.group.set_planner_id("RRTConnectkConfigDefault")
		self.group.set_planning_time(5.0)
		self.group.set_num_planning_attempts(10)
		self.group.set_max_velocity_scaling_factor(0.1)
		self.group.set_max_acceleration_scaling_factor(0.1)

		self.ee_pose = Pose()
		self.plan = moveit_msgs.msg.RobotTrajectory()

	def ee_pose_update_cb(self, msg):
		if msg.poses:
			cur_pose = msg.poses[0].pose
			# print("="*80)
			# print(cur_pose)
			self.ee_pose.position.x = cur_pose.position.x
			self.ee_pose.position.y = cur_pose.position.y
			self.ee_pose.position.z = cur_pose.position.z

			self.ee_pose.orientation.x = cur_pose.orientation.x
			self.ee_pose.orientation.y = cur_pose.orientation.y
			self.ee_pose.orientation.z = cur_pose.orientation.z
			self.ee_pose.orientation.w = cur_pose.orientation.w

	def ur5_plan_cb(self, msg):
		self.group.set_pose_target(self.ee_pose)
		self.plan = self.group.plan()
		rospy.loginfo("ur5_plan: Waiting while RVIZ displays the plan...")
		rospy.sleep(5)
		rospy.loginfo("ur5_plan: Visualizing the plan")
		self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		self.display_trajectory.trajectory_start = self.robot.get_current_state()
		self.display_trajectory.trajectory.append(self.plan)
		self.display_trajectory_publisher.publish(self.display_trajectory);
		rospy.loginfo("ur5_plan: Waiting while plan is visualized (again)...")
		rospy.sleep(5)
		self.group.clear_pose_targets()
		rospy.loginfo("ur5_plan: Finished")

	def ur5_execute_cb(self, msg):
		rospy.loginfo("ur5_execute: The Plan Execution Started")
		self.group.execute(self.plan, wait=False)
		rospy.sleep(5)
		rospy.loginfo("ur5_execute: Finished")

	def ur5_stop_cb(self, msg):
		self.group.stop()
		rospy.loginfo("ur5_stop: Stopped")

	# def ur5_reset_cb(self, msg):
		


def main():
	try:
		ur5 = UR5MoveGroupGUI()
		while not rospy.is_shutdown():
			rospy.sleep(10)

	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()