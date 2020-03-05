#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import *

from const import *

class PointCloudGUI():
	def __init__(self):
		rospy.init_node("pcl_capture")
		
		# Subscriber
		rospy.Subscriber("/pcl_capture", Bool, self.pcl_capture_cb)
		rospy.Subscriber("/pcl_clear", Bool, self.pcl_clear_cb)	

		# Publisher
		self.captured_pcl_pub = rospy.Publisher('/captured_pcl', PointCloud2, queue_size=1)
		self.fake_point_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
		self.instruction_pub = rospy.Publisher('/instruction', String, queue_size=1)

		# Parameter
		if not rospy.has_param("~captured_cloud_in"):
			rospy.set_param("~captured_cloud_in", "/pcl_fusion_node_color/pcl_fusion_node/fused_points")
		self.pcl_in = rospy.get_param("~captured_cloud_in")

		self.last_pcl = None
		self.last_pcl_len = 0

	def pcl_cb(self, msg):
		rospy.logdebug("got point cloud msg")
		self.captured_pcl_pub.publish(msg)

	def pcl_capture_cb(self, msg):
		if msg.data:
			rospy.loginfo("clicked pcl capture | {}".format(self.pcl_in))

			captured_pcl = rospy.wait_for_message(self.pcl_in,
                                PointCloud2, 60)
			
			self.captured_pcl_pub.publish(captured_pcl)

			self.last_pcl = captured_pcl
			self.last_pcl_len = len(captured_pcl.data)

			self.instruction_pub.publish(STEP2)
		else:
			time_now = rospy.Time.now()
			print("pcl_capture_cb==============started")
			print(time_now)
			self.last_pcl.header.stamp = time_now
			self.captured_pcl_pub.publish(self.last_pcl)
			print("pcl_capture_cb==============ended")

	def pcl_clear_cb(self, msg):
		rospy.loginfo("clicked pcl clear")
		if self.last_pcl is not None:
			self.last_pcl.header.stamp = rospy.Time.now()
			self.last_pcl.data = [0] * self.last_pcl_len

			self.captured_pcl_pub.publish(self.last_pcl)

			fake_point = PointStamped()
			fake_point.header.frame_id = FRAME_ID
			fake_point.point.z = 10 
			self.fake_point_pub.publish(fake_point) 

if __name__ == '__main__':
	pcl = PointCloudGUI()
	rospy.spin()
