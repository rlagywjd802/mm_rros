#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2

class PointCloudGUI():
	def __init__(self):
		rospy.loginfo("pcl init")
		rospy.init_node('mm_pcl_gui_execution', anonymous=True)
		
		# Subscriber
		# rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pcl_cb)
		rospy.Subscriber("/pcl_capture", Bool, self.pcl_capture_cb)
		rospy.Subscriber("/pcl_clear", Bool, self.pcl_clear_cb)	

		# Publisher
		self.new_pcl_pub = rospy.Publisher('/captured_pcl', PointCloud2, queue_size=1)

		self.pcl_data = PointCloud2()
		self.pcl_data_len = 0

	def pcl_cb(self, msg):
		rospy.loginfo("got point cloud msg")
		self.new_pcl_pub.publish(msg)	

	def pcl_capture_cb(self, msg):
		rospy.loginfo("clicked pcl capture")
		self.pcl_data = rospy.wait_for_message("/camera/depth/color/points", PointCloud2, 20)
		self.pcl_data_len = len(self.pcl_data.data)
		self.new_pcl_pub.publish(self.pcl_data)

	def pcl_clear_cb(self, msg):
		rospy.loginfo("clicked pcl clear")
		if self.pcl_data_len != 0:
			self.pcl_data.data = [0] * self.pcl_data_len
			self.new_pcl_pub.publish(self.pcl_data)

if __name__ == '__main__':
	pcl = PointCloudGUI()
	rospy.spin()