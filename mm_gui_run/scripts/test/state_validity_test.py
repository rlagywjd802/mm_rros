#!/usr/bin/env python

import rospy
import moveit_msgs.msg

from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

JOINTS_NAME = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class StateValidity():
    def __init__(self):

        # subscribe to robot state
        rospy.Subscriber("/inv_kin_sol", moveit_msgs.msg.DisplayRobotState, self.robot_state_cb, queue_size=1)

        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

        # wait for service to become available
        self.sv_srv.wait_for_service()
        rospy.loginfo('service is avaiable')

        # prepare msg to interface with moveit
        self.rs = moveit_msgs.msg.DisplayRobotState()

    def checkCollision(self):
        '''
        check if robotis in collision
        '''
        if self.getStateValidity().valid:
            rospy.loginfo('robot not in collision, all ok!')
        else:
            rospy.logwarn('robot in collision')


    def robot_state_cb(self, msg):
        '''
        update robot state
        '''
        self.rs = msg

    def getStateValidity(self, group_name='ur5', constraints=None):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.rs.state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        print("====================================")
        print(self.rs.state.joint_state.position)

        result = self.sv_srv.call(gsvr)

        print('result:')
        print('valid = {}'.format(result.valid))
        rc = result.contacts
        for i in range(len(rc)):
            print('contact {}: 1-{}, 2-{}'.format(i, rc[i].contact_body_1, rc[i].contact_body_2))

        return result

    def start_collision_checker(self):
        while not rospy.is_shutdown():
            self.checkCollision()
            rospy.Rate(1).sleep()  

if __name__ == '__main__':
    rospy.init_node('collision_checker_node', anonymous=False)
    collision_checker_node = StateValidity()
    collision_checker_node.start_collision_checker()