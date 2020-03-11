#!/usr/bin/env python
import rospy

from actionlib_msgs.msg import GoalID

rospy.init_node('test_mb_cancel')

mb_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

for i in range(3):
	cancel_msg = GoalID()
	# cancel_msg.id = "/move_base-3-1583528039.114547371"
	cancel_msg.stamp = rospy.Time.now()

	mb_cancel_pub.publish(cancel_msg)
	rospy.sleep(1)
	print("published")