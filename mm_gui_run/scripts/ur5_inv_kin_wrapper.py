#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

import moveit_msgs.msg
import std_msgs.msg

from ur5_inv_kin import ur5
from const import *

JOINTS_NAME = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
				'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
LINKS_NAME = ['base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 
				'wrist_1_link', 'wrist_2_link', 'wrist_3_link']

J1 = [-360.0, -60.0]	# (-180) +120, -180  
J2 = [-240.0, 60.0] 	# (-90) +-150
J3 = [-360.0, 360.0]
J4 = [-360.0, 0.0]
J5 = [-120.0, -60.0]	# (-90) += 30
J6 = [-270.0, 90.0]		# (-90) += 180

rad2deg = 180.0/math.pi
deg2rad = math.pi/180.0

class ur5_inv_kin_wrapper(ur5):
	def __init__(self):
		ur5.__init__(self)
		self.inv_sol = None
		self.robot_state_pub = rospy.Publisher('/inv_kin_sol', moveit_msgs.msg.DisplayRobotState, queue_size=1)
		self.cost_pub = rospy.Publisher('/inv_kin_cost', std_msgs.msg.Float32MultiArray, queue_size=1)
		self.w = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

	def _convert_base_axis(self, pose_mat):
		# pose_mat : [4x4] array
		# /base_link is rotated 180 deg along z axis compared to real robot
		z_180 = tf.transformations.euler_matrix(0, 0, math.pi)
		pose_mat = tf.transformations.concatenate_matrices(z_180, pose_mat)

		return pose_mat

	def _tf_to_mat(self, trans, rot):
		# pose_mat : [4x4] array
		trans_mat = tf.transformations.translation_matrix(trans)
		rot_mat = tf.transformations.quaternion_matrix(rot)
		pose_mat = tf.transformations.concatenate_matrices(trans_mat, rot_mat)

		pose_mat = self._convert_base_axis(pose_mat)
		return pose_mat

	def _print_sol(self, sol_rad):
		print('-'*50)
		for i in range(8):
			sol_deg = sol_rad[:, i] * rad2deg
			print("sol {} : [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(
				i, sol_deg[0], sol_deg[1], sol_deg[2], sol_deg[3], sol_deg[4], sol_deg[5]))

	def _inv_kin_limited(self, pose_mat):
		inv_sol = self.inv_kin(pose_mat)
		for i in range(8):
			if inv_sol[0, i] > (J1[0]+360.0)*deg2rad:
				inv_sol[0, i] -= math.pi*2
			# if inv_sol[1, i] > J2[1]*deg2rad:
			# 	inv_sol[1, i] -= math.pi*2
			if inv_sol[3, i] > J4[1]*deg2rad:
				inv_sol[3, i] -= math.pi*2
			# if inv_sol[5, i] > J6[1]*deg2rad:
			# 	inv_sol[5, i] -= math.pi*2
		# self._print_sol(inv_sol)
		return inv_sol

	def _calculate_cost_(self, inv_sol, cur_joint):
		costs = []
		for i in range(8):
			cost = 0.0
			for j in range(6): 
				cost += self.w[j] * (inv_sol[j][i] - cur_joint[j])**2 
			costs.append(cost)
		print('costs: {}'.format(costs))

		return costs

	# def _calculate_cost(self, cur_joint):
	# 	costs = []
	# 	for i in range(8):
	# 		cost = 0.0
	# 		for j in range(6): 
	# 			cost += self.w[j] * (self.inv_sol[j][i] - cur_joint[j])**2 
	# 		costs.append(cost)
	# 	print('costs: {}'.format(costs))

		return costs

	def solve_(self, trans, rot, cur_joint):
		pose_mat = self._tf_to_mat(trans, rot)
		inv_sol = self._inv_kin_limited(pose_mat)
		cost = self._calculate_cost_(inv_sol, cur_joint)

		inv_sol_list = []
		for i in range(8):
			inv_sol_list.append((cost[i], inv_sol[:, i]))
		sorted_list = sorted(inv_sol_list)
		sorted_inv_sol = np.zeros((6, 8))
		for i in range(8):
			for j in range(6):
				val = sorted_list[i][1]
				sorted_inv_sol[j][i] = (float(val[j]))
		self._print_sol(sorted_inv_sol)
		self.inv_sol = sorted_inv_sol


	# def solve(self, trans, rot):
	# 	now = rospy.Time.now()
	# 	pose_mat = self._tf_to_mat(trans, rot)
	# 	self.inv_sol = self._inv_kin_limited(pose_mat)
	# 	# print(str((rospy.Time.now().nsecs - now.nsecs)/1000000.0)+" ms")

	def publish_cost(self, cur_joint):
		inv_sol_cost = self._calculate_cost(cur_joint)

		msg = std_msgs.msg.Float32MultiArray()
		msg.data = inv_sol_cost

		self.cost_pub.publish(msg)

	def publish_state(self, num):
		print("publish_state===================================================")
		link_colors = []
		if num == -1:
			selected_inv_sol = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			for i in range(len(LINKS_NAME)):
				color = moveit_msgs.msg.ObjectColor()
				color.id = LINKS_NAME[i]
				color.color.a = 0.0
				link_colors.append(color)
		else:
			selected_inv_sol = self.inv_sol[:, num]
			for i in range(len(LINKS_NAME)):
				color = moveit_msgs.msg.ObjectColor()
				color.id = LINKS_NAME[i]
				color.color.g = 20.0
				color.color.a = 0.7
				link_colors.append(color)

		print(selected_inv_sol)

		robot_state = moveit_msgs.msg.DisplayRobotState()
		robot_state.state.joint_state.header.frame_id = FRAME_ID
		robot_state.state.joint_state.name = JOINTS_NAME
		robot_state.state.joint_state.position = selected_inv_sol
		robot_state.state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		robot_state.state.multi_dof_joint_state.header.frame_id = FRAME_ID
		robot_state.highlight_links = link_colors

		self.robot_state_pub.publish(robot_state)

		return selected_inv_sol

def main():
	rospy.init_node('ur5_inv_kin_wrapper')
	
	ur5_inv = ur5_inv_kin_wrapper()

	listener = tf.TransformListener()
	
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/base_link', REAL_EEF_LINK, rospy.Time(0))
			ur5_inv.solve(trans, rot)
			ur5_inv.publish_state(0)
		except Exception as e:
			print(e)
			continue
		print('='*50)
		rospy.Rate(1).sleep()	

if __name__ == '__main__':
	main()