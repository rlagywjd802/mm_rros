#!/usr/bin/env python
import sys
import copy
import rospy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose, PointStamped, PoseStamped
from visualization_msgs.msg import InteractiveMarkerUpdate
from sensor_msgs.msg import Joy

from utils import *
from const import *

class UR5MoveGroupGUI():
    def __init__(self, log_level):
        rospy.init_node('mm_moveit_gui_execution', log_level=log_level)

        # Subscriber
        # rospy.Subscriber("clicked_point", PointStamped, self.clicked_cb)

        rospy.Subscriber("approach_plan", Bool, self.approach_plan_cb)
        rospy.Subscriber("approach_execute", Bool, self.approach_execute_cb)  
        rospy.Subscriber("approach_stop", Bool, self.approach_stop_cb)

        rospy.Subscriber("move_xp", Bool, self.move_xp_cb)
        rospy.Subscriber("move_xm", Bool, self.move_xm_cb)
        rospy.Subscriber("move_yp", Bool, self.move_yp_cb)
        rospy.Subscriber("move_ym", Bool, self.move_ym_cb)
        rospy.Subscriber("move_zp", Bool, self.move_zp_cb)
        rospy.Subscriber("move_zm", Bool, self.move_zm_cb)

        rospy.Subscriber("waypoints/update", InteractiveMarkerUpdate, self.waypoints_update_cb)
        rospy.Subscriber("rotate_axis", String, self.rotate_axis_cb)
        rospy.Subscriber("distance", Int32, self.distance_cb)

        # Publisher
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=50)

        self.instruction_pub = rospy.Publisher('/instruction', String, queue_size=1)
        
        # moveit commander
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        # self.scene.attach_mesh("base_footprint", 'base_scene', filename='./base_scene.STL') ## doesn't work
        
        self.group = moveit_commander.MoveGroupCommander("ur5")         
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)

        self.last_waypoints = []
        self.ee_pose = Pose()

        self.plan = moveit_msgs.msg.RobotTrajectory()

        self.last_offset = 10.0

        self.listener = tf.TransformListener()


    ##################################################################################################
    ##################################################################################################

    def go(self, dx, dy, dz):
        current_pose = self.group.get_current_pose().pose
        pose_goal = Pose()
        pose_goal.position.x = current_pose.position.x + dx
        pose_goal.position.y = current_pose.position.y + dy
        pose_goal.position.z = current_pose.position.z + dz
        pose_goal.orientation = current_pose.orientation
        
        self.group.set_pose_target(pose_goal)
        self.group.go(wait=True)
        self.group.clear_pose_targets()
        # rospy.sleep(5)
        rospy.loginfo("move cb go: Finished")

    ##################################################################################################
    ##################################################################################################

    def move_xp_cb(self, msg):
        rospy.loginfo("move_xp_cb")
        # self.go(0.01, 0, 0)
        self.go(0, -0.01, 0)

    def move_xm_cb(self, msg):
        rospy.loginfo("move_zm_cb")
        # self.go(-0.01, 0, 0)
        self.go(0, 0.01, 0)

    def move_yp_cb(self, msg):
        rospy.loginfo("move_yp_cb")
        # self.go(0, 0.01, 0)
        self.go(0.01, 0, 0)

    def move_ym_cb(self, msg):
        rospy.loginfo("move_ym_cb")
        # self.go(0, -0.01, 0)
        self.go(-0.01, 0, 0)

    def move_zp_cb(self, msg):
        rospy.loginfo("move_zp_cb")
        self.go(0, 0, 0.01)

    def move_zm_cb(self, msg):
        rospy.loginfo("move_zm_cb")
        self.go(0, 0, -0.01)

    # def clicked_cb(self, msg):
    #     rospy.loginfo("clicked [x, y, z]: "+str(msg.point.x)+","+str(msg.point.y)+","+str(msg.point.z))
    #     self.ee_pose.position.x = msg.point.x
    #     self.ee_pose.position.y = msg.point.y
    #     self.ee_pose.position.z = msg.point.z + 0.2

    #     current_pose_orientation = self.group.get_current_pose().pose.orientation
    #     print(current_pose_orientation)
    #     self.ee_pose.orientation = current_pose_orientation

    def approach_plan_cb(self, msg):
        self.group.set_pose_target(self.last_waypoints[0])
        self.plan = self.group.plan()
        rospy.loginfo("approach_plan: Waiting while RVIZ displays the plan...")
        rospy.sleep(5)
        rospy.loginfo("approach_plan: Visualizing the plan")
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(self.plan)
        self.display_trajectory_publisher.publish(self.display_trajectory);
        rospy.loginfo("approach_plan: Waiting while plan is visualized (again)...")
        rospy.sleep(5)
        self.group.clear_pose_targets()
        rospy.loginfo("approach_plan: Finished")

    def approach_execute_cb(self, msg):
        rospy.loginfo("approach_execute: The Plan Execution Started")
        self.group.execute(self.plan, wait=False)
        rospy.sleep(5)
        rospy.loginfo("approach_execute: Finished")

        self.instruction_pub.publish(STEP4)


    def approach_stop_cb(self, msg):
        self.group.stop()
        rospy.loginfo("approach_stop: Stopped")

    def waypoints_update_cb(self, msg):
        rospy.logdebug("waypoints_update_cb")
        if msg.poses:
            # listen tf of 'target_pose' 
            try:
                (trans, rot) = self.listener.lookupTransform(FRAME_ID, 'gripper_pose', rospy.Time(0))
                rospy.logdebug("waypoints_update_cb| trans:{}, rot:{}".format(trans, rot))

                self.last_waypoints = []
                for i in reversed(range(len(msg.poses))):
                    if i == len(msg.poses)-1:
                        target_pose = Pose()
                        target_pose.position = Point(trans[0], trans[1], trans[2])
                        target_pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
                    else:
                        target_pose = msg.poses[i].pose
                    rospy.logdebug("waypoints_update_cb| marker{} ".format(i)+print_pose(msg.poses[i].pose))
                    self.last_waypoints.append(target_pose)
            except:
                pass

    def rotate_axis_cb(self, msg):
        rotate_axis = msg.data

        rospy.loginfo("rotate_axis_cb| msg: {}".format(rotate_axis))
        self.last_r_axis = rotate_axis

    def distance_cb(self, msg):
        offset = msg.data

        rospy.loginfo("distance_cb| msg: {}".format(offset))
        self.last_offset = offset

def main(arg):
    if len(arg) > 1:
        if arg[1] == "debug":
            log_level = rospy.DEBUG
    else:
        log_level = rospy.INFO
    try:
        ur5 = UR5MoveGroupGUI(log_level)
        while not rospy.is_shutdown():
            rospy.sleep(10)

    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main(sys.argv)