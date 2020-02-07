#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
import numpy as np
import math

from ur5_inv_kin import ur5
from const import *

def convert_eef_axis(rot_mat):
	# convert eef axis of urdf to real robot
	r_x = rot_mat[:3, 0]
	r_y = rot_mat[:3, 1]
	r_z = rot_mat[:3, 2]
	rot_mat[:3, 0] = -r_y
	rot_mat[:3, 1] = -r_z
	rot_mat[:3, 2] = np.flip(r_x)
	rot_mat[0, 2] = -rot_mat[0, 2]
	return rot_mat

def convert_base_axis(pose_mat):
	z_180 = tf.transformations.euler_matrix(0, 0, math.pi)
	pose_mat = tf.transformations.concatenate_matrices(z_180, pose_mat)
	
	return pose_mat

ur5_test = ur5()

rospy.init_node('ur5_inv_kin_test')

listener = tf.TransformListener()

group = moveit_commander.MoveGroupCommander(MOVE_GROUP)
eef_link = group.get_end_effector_link()
print('eef_link: {}'.format(eef_link))

while not rospy.is_shutdown():
	# subscribe tf
	try:
		(trans, rot) = listener.lookupTransform('/base_link', '/real_eef_pose', rospy.Time(0))
		# (trans, rot) = listener.lookupTransform('/base_link', '/real_ee_link', rospy.Time(0))
	except:
		continue

	cur_joint = group.get_current_joint_values()
	print('cur_joint :')
	print(cur_joint)
	print("============================================")

	# cur_pose = group.get_current_pose().pose
	# print('cur_pose :')
	# print(cur_pose)

	# cp = cur_pose.position
	# co = cur_pose.orientation
	# trans_mat = tf.transformations.translation_matrix([cp.x, cp.y, cp.z])
	# rot_mat = tf.transformations.quaternion_matrix([co.x, co.y, co.z, co.w])
	
	trans_mat = tf.transformations.translation_matrix(trans)
	rot_mat = tf.transformations.quaternion_matrix(rot)

	pose_mat = tf.transformations.concatenate_matrices(trans_mat, rot_mat)
	pose_mat = convert_base_axis(pose_mat)

	# pose_mat = convert_base_to_basefootprint(pose_mat)

	print('pose_mat: ')
	print(pose_mat)
	print("============================================")

	cur_joint_fk = ur5_test.fwd_kin(cur_joint)
	print('cur_joint_fk :')
	print(cur_joint_fk)
	print("============================================")

	cur_pose_inv = ur5_test.inv_kin(pose_mat)

	num_of_sol = len(cur_pose_inv[0])
	sol = []
	print('num of sol = {}'.format(num_of_sol))
	print('inv sol:')
	for i in range(num_of_sol):
		sol.append(cur_pose_inv[:, i])
		print(sol[i])
	print("============================================")

	robot_state_pubs = []
	robot_states = []

	for i in range(num_of_sol):
		robot_state_pub = rospy.Publisher('/ik_solution'+str(i), moveit_msgs.msg.DisplayRobotState, queue_size=1)

		robot_state = moveit_msgs.msg.DisplayRobotState()
		robot_state.state.joint_state.header.frame_id = FRAME_ID
		robot_state.state.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
		  'wrist_3_joint']
		robot_state.state.joint_state.position = sol[i]
		robot_state.state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		robot_state.state.multi_dof_joint_state.header.frame_id = FRAME_ID

		robot_state_pubs.append(robot_state_pub)
		robot_states.append(robot_state)

	for i in range(num_of_sol):
		robot_state_pubs[i].publish(robot_states[i])
	print('published')
	rospy.Rate(1).sleep()