#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from visualization_msgs.msg import *

def pcl_capture_cb(msg):
	global is_captured

	if msg.data:
		rospy.loginfo("clicked pcl capture")
		is_captured = True

def create_marker():
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.header.stamp = rospy.Time.now()

	marker.type = Marker.CUBE

	marker.pose.position.x = -0.1
	marker.pose.position.y = -2.0
	marker.pose.position.z = 1.0
	marker.scale.x = 0.5
	marker.scale.y = 0.5
	marker.scale.z = 0.5
	marker.color.r = 0
	marker.color.g = 255
	marker.color.b = 0
	marker.color.a = 0.5

	return marker

def delete_marker():
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.header.stamp = rospy.Time.now()

	marker.type = Marker.CUBE

	return marker


if __name__ == '__main__':

	is_captured = False

	rospy.init_node("marker_publish")
	marker_pub = rospy.Publisher("marker_test", Marker, queue_size=1)
	rospy.Subscriber("pcl_capture", Bool, pcl_capture_cb)

	marker = create_marker()

	while not rospy.is_shutdown():
		if is_captured:
			marker_pub.publish(delete_marker())
			break
		marker_pub.publish(marker)
		print "published"
		rospy.Rate(1).sleep()