import tf

from geometry_msgs.msg import *

def euler_to_quat(r, p, y):
	quat_array = tf.transformations.quaternion_from_euler(r, p, y)
	
	quat = Quaternion()
	quat.x = quat_array[0]
	quat.y = quat_array[1]
	quat.z = quat_array[2]
	quat.w = quat_array[3]

	return quat

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