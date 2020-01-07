#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

class JoysticCommand():
	def __init__(self):
		rospy.loginfo("joy init")
		rospy.init_node('mm_demo_joy_execution', anonymous=True)
		rospy.Subscriber("/joy", Joy, self.joy_cb)

	def joy_cb(self, msg):
		rospy.loginfo("joy cb")
		x = msg.axes[1]
		y = msg.axes[0]
		z_up_abs = abs(msg.axes[4])
		z_down_abs = abs(msg.axes[6])
		enable = msg.buttons[3]

		# if(z_up_abs >= z_down_abs):
		# 	z = z_up_abs
		# else:
		# 	z = -z_down_abs
		z = z_up_abs if (z_up_abs >= z_down_abs) else -z_down_abs

		x = x if enable == 1 else 0
		y = y if enable == 1 else 0
		z = z if enable == 1 else 0
		rospy.loginfo(str(enable)+": "+str(x)+", "+str(y)+", "+str(z))

if __name__ == '__main__':
	joy = JoysticCommand()
	rospy.spin()