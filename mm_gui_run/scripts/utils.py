import tf
import numpy as np

from const import *
from geometry_msgs.msg import *

# transformation related
def tr_to_mat(trans, rot):
	trans_mat = tf.transformations.translation_matrix(trans)
	rot_mat = tf.transformations.quaternion_matrix(rot)
	pose_mat = tf.transformations.concatenate_matrices(trans_mat, rot_mat)

	return pose_mat

def mat_to_tr(pose_mat):
	trans = tf.transformations.translation_from_matrix(pose_mat)
	rot = tf.transformations.quaternion_from_matrix(pose_mat)

	return trans, rot

def pose_to_mat(pose):
	pp = pose.position
	po = pose.orientation
	trans_mat = tf.transformations.translation_matrix([pp.x, pp.y, pp.z])
	rot_mat = tf.transformations.quaternion_matrix([po.x, po.y, po.z, po.w])
	pose_mat = tf.transformations.concatenate_matrices(trans_mat, rot_mat)
	
	return pose_mat

def mat_to_pose(pose_mat):
	trans = tf.transformations.translation_from_matrix(pose_mat)
	rot = tf.transformations.quaternion_from_matrix(pose_mat)

	pose = Pose()
	pose.position.x = trans[0]
	pose.position.y = trans[1]
	pose.position.z = trans[2]
	pose.orientation.x = rot[0]
	pose.orientation.y = rot[1]
	pose.orientation.z = rot[2]
	pose.orientation.w = rot[3]

	return pose

def euler_to_quat(r, p, y):
	quat_array = tf.transformations.quaternion_from_euler(r, p, y)
	
	quat = Quaternion()
	quat.x = quat_array[0]
	quat.y = quat_array[1]
	quat.z = quat_array[2]
	quat.w = quat_array[3]

	return quat

def transform_pose_to_pose(a_pose, T_trans):
	# transfrom pose(point, quat) to pose : b = T*R*a
	ap = a_pose.position
	ao = a_pose.orientation
	a_point = [ap.x, ap.y, ap.z]
	a_quat = [ao.x, ao.y, ao.z, ao.w]
	# a_point = a_pose[:3]
	# a_quat = a_pose[3:]

	# 1. a_quat, a_point -> a_mat
	a_mat = tf.transformations.identity_matrix()
	a_mat[:3, 3] = tf.transformations.translation_matrix(a_point)[:3, 3]
	a_mat[:3, :3] = tf.transformations.quaternion_matrix(a_quat)[:3, :3]
	# print(a_mat)

	# 2. R_rot -> R_mat
	R_rot = [INIT_ORIENT.x, INIT_ORIENT.y, INIT_ORIENT.z, INIT_ORIENT.w]
	R_mat = tf.transformations.quaternion_matrix(R_rot)
	# print(R_mat)

	# 3. T_trans -> T_mat
	T_mat = tf.transformations.translation_matrix(T_trans)
	# print(T_mat)

	# 3. b_mat = T_mat*R_mat*a_mat
	b_mat = tf.transformations.concatenate_matrices(T_mat, R_mat, a_mat)
	# print(b_mat)

	# 4. b_mat -> b_quat, b_point
	b_point = tf.transformations.translation_from_matrix(b_mat)
	b_quat = tf.transformations.quaternion_from_matrix(b_mat)

	b_pose = Pose()
	b_pose.position.x = b_point[0]
	b_pose.position.y = b_point[1]
	b_pose.position.z = b_point[2]
	b_pose.orientation.x = b_quat[0]
	b_pose.orientation.y = b_quat[1]
	b_pose.orientation.z = b_quat[2]
	b_pose.orientation.w = b_quat[3]

	return b_pose

## print functions
def print_pose(pose, name="pose: "):
    pp = pose.position
    po = pose.orientation
    
    print_msg = name
    print_msg += "{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(
        pp.x, pp.y, pp.z, po.x, po.y, po.z, po.w)

    return print_msg

def print_point(point, name="point: "):    
    print_msg = name
    print_msg += "{:.2f}, {:.2f}, {:.2f}".format(point.x, point.y, point.z)

    return print_msg

if __name__== "__main__":
	offset = -0.1

	a_pose = [-0.35, -0.59, 0.78, 0.00, -0.00, -0.50, 0.87]
	T_trans = [offset, 0, 0]

	b_pose = transform_pose_to_pose(a_pose, T_trans)
	print(b_pose)
