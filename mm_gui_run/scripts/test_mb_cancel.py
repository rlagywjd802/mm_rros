#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult

class MoveBaseControl():
	def __init__(self):
		rospy.init_node('test_mb_cancel')

		feedback_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.feedback_cb)
		result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_cb)
		click_cancel_sub = rospy.Subscriber('/clicked_cancel', Bool, self.click_cancel_cb)
		self.mb_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
		self.last_goal_id = None

	def result_cb(self, msg):
		result_goal_id = msg.status.goal_id.id
		if result_goal_id == self.last_goal_id:
			if msg.status.status == 3:
				print "goal reached"
			if msg.status.status == 4:
				print "plan canceled because of oscilation"
		else:
			print result_goal_id

	def feedback_cb(self, msg):
		self.last_goal_id = msg.status.goal_id.id
		# print self.last_goal_id

	def click_cancel_cb(self, msg):
		if msg.data:
			print "plan canceled because of user inputs"
			for i in range(3):
				cancel_msg = GoalID()
				cancel_msg.id = self.last_goal_id
				cancel_msg.stamp = rospy.Time.now()
				self.mb_cancel_pub.publish(cancel_msg)
				# print "move base cancel published"


# for i in range(3):
# 	cancel_msg = GoalID()
# 	# cancel_msg.id = "/move_base-3-1583528039.114547371"
# 	cancel_msg.stamp = rospy.Time.now()

# 	# mb_cancel_pub.publish(cancel_msg)
# 	# rospy.sleep(1)
# 	# print("published")

def main():
	test = MoveBaseControl()
	rospy.spin()

if __name__ == '__main__':
    main()