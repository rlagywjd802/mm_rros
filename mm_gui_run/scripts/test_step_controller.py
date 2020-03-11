#!/usr/bin/env python
import rospy

from std_msgs.msg import Int32

rospy.init_node('test_step_controller')
mb_step_pub = rospy.Publisher('/mm_gui_step', Int32, queue_size=1)
r = rospy.Rate(10)

while not rospy.is_shutdown():
	step_msg = Int32()
	mb_step_pub.publish(step_msg)
	print("published")
	r.sleep()