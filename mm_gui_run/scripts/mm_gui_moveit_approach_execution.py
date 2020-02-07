#!/usr/bin/env python
import sys
import copy
import rospy
import tf

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose, PointStamped, PoseStamped
from visualization_msgs.msg import InteractiveMarkerUpdate
from sensor_msgs.msg import Joy

from std_srvs.srv import Trigger

from ur5_inv_kin_wrapper import ur5_inv_kin_wrapper
from utils import *
from const import *

DXYZ = 0.02

# fixed motion for record
initial_joint = [-1.6746166388141077, -1.0553820768939417, -1.9730132261859339, -1.5056265036212366, 0.8055320382118225, -3.3957682291613978]
joint1 = [-1.7512124220477503, -0.7758315245257776, -2.0813515822040003, -1.6075804869281214, 0.821963906288147, -3.4971850554095667]
joint2 = [-2.114042107258932, -0.5674937407123011, -2.1670878569232386, -1.456916634236471, 0.9604483842849731, -3.929640833531515]
joint3 = [-3.0713780562030237, -0.7171343008624476, -2.2055943647967737, -1.0137255827533167, 1.572555422782898, -4.720593277608053]
joint4 = [-3.5758140722857874, -0.9128931204425257, -2.144115749989645, -0.9496153036700647, 1.9175775051116943, -5.097561899815695]
joint5 = [-3.9125710169421595, -1.1535161177264612, -2.010705296193258, -0.9759872595416468, 2.120448350906372, -5.39805776277651]
final_joint = [-4.152798000966207, -1.4738629500018519, -1.7466023604022425, -1.0692537466632288, 2.235809326171875, -5.655894641076223]
record_motion = [initial_joint, joint1, joint2, joint3, joint4, joint5, final_joint]

def dxyz_wrt_eef_pose(cur_pose, dxyz):
    cur_pose_mat = pose_to_mat(cur_pose)
    dpose_mat = tf.transformations.translation_matrix(dxyz)
    pose_mat = tf.transformations.concatenate_matrices(cur_pose_mat, dpose_mat)
    pose = mat_to_pose(pose_mat)

    return pose

def dxyz_wrt_eef_tr(trans, rot, dxyz):
    cur_pose_mat = tr_to_mat(trans, rot)
    dpose_mat = tf.transformations.translation_matrix(dxyz)
    pose_mat = tf.transformations.concatenate_matrices(cur_pose_mat, dpose_mat)
    (trans, mat) = mat_to_tr(pose_mat)

    return trans, mat

def pcl_fusion_reset():
    try:
        reset_pcl = rospy.ServiceProxy('/pcl_fusion_node/reset', Trigger)
        resp = reset_pcl()
        if resp.success:
            rospy.loginfo("pcl_fusion_reset| reset success")
            return True
        else:
            return False
    except Exception as e:
        rospy.loginfo(e)
        return False

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
        rospy.Subscriber("distance", Int32, self.distance_cb)

        rospy.Subscriber("solution_num", Int32, self.solution_cb)

        rospy.Subscriber("gripper_close", Bool, self.gripper_close_cb)

        rospy.Subscriber("pcl_record", Bool, self.record_move_cb)
        rospy.Subscriber("compute_interpolation", Bool, self.stitch_plan_cb)

        # Publisher
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=50)

        self.instruction_pub = rospy.Publisher('/instruction', String, queue_size=1)

        self.aco_pub = rospy.Publisher('/attached_collision_object', moveit_msgs.msg.AttachedCollisionObject, queue_size=100)
        self.captured_pcl = rospy.Publisher('/pcl_capture', Bool, queue_size=1)

        # Service
        rospy.wait_for_service('/pcl_fusion_node/reset')
        
        # ready for listen tf
        self.listener = tf.TransformListener()

        # moveit commander
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.group = moveit_commander.MoveGroupCommander("ur5")         
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)
        self.group.set_max_velocity_scaling_factor(0.05)        # 0.1
        self.group.set_max_acceleration_scaling_factor(0.05)    # 0.1

        self.last_waypoints = []

        self.plan = moveit_msgs.msg.RobotTrajectory()

        self.last_offset = 10.0

        # IK
        self.ur5_inv = ur5_inv_kin_wrapper()
        # self.ur5_inv.publish_state(-1) ## doesn't work
        self.last_target_joint = None
        self.last_sol_num = None

        self.count = 0


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

    def move_to_target_joint(self, joint):
        self.group.set_joint_value_target(joint)
        self.group.go(wait=True)
        self.group.clear_pose_targets()
        rospy.sleep(1)

    def move_to_target_pose(self, pose):
        self.group.set_pose_target(pose)
        self.group.go(wait=True)
        self.group.clear_pose_targets()

    def move_dxyz(self, dxyz):
        (trans, rot) = self.listener.lookupTransform('/base_link', '/real_ee_link', rospy.Time(0))
        rospy.loginfo("trans:{}, rot:{}".format(trans, rot))

        (dtrans, drot) = dxyz_wrt_eef_tr(trans, rot, dxyz)

        current_joint = self.group.get_current_joint_values()
        self.ur5_inv.solve_and_sort(dtrans, drot, current_joint)

        valid, target_joint = self.ur5_inv.publish_state(0)
        rospy.sleep(1)
        if valid:
            self.move_to_target_joint(target_joint)

    def move_xp_cb(self, msg):
        rospy.loginfo("move_xp_cb")
        self.move_dxyz([0, 0, DXYZ])

    def move_xm_cb(self, msg):
        rospy.loginfo("move_xm_cb")
        self.move_dxyz([0, 0, -DXYZ])

    def move_yp_cb(self, msg):
        rospy.loginfo("move_yp_cb")
        self.move_dxyz([0, DXYZ, 0])

    def move_ym_cb(self, msg):
        rospy.loginfo("move_ym_cb")
        self.move_dxyz([0, -DXYZ, 0])

    def move_zp_cb(self, msg):
        rospy.loginfo("move_yp_cb")
        self.move_dxyz([-DXYZ, 0, 0])

    def move_zm_cb(self, msg):
        rospy.loginfo("move_ym_cb")
        self.move_dxyz([DXYZ, 0, 0])

    # def move_xp_cb(self, msg):
    #     rospy.loginfo("move_xp_cb")
    #     cur_pose = self.group.get_current_pose().pose
    #     des_pose = dxyz_wrt_eef_pose(cur_pose, [DXYZ, 0, 0])
    #     self.move_to_target_pose(des_pose)

    # def move_xm_cb(self, msg):
    #     rospy.loginfo("move_xm_cb")
    #     cur_pose = self.group.get_current_pose().pose
    #     des_pose = dxyz_wrt_eef_pose(cur_pose, [-DXYZ, 0, 0])
    #     self.move_to_target_pose(des_pose)

    # def move_yp_cb(self, msg):
    #     rospy.loginfo("move_yp_cb")
    #     cur_pose = self.group.get_current_pose().pose
    #     des_pose = dxyz_wrt_eef_pose(cur_pose, [0, 0, -DXYZ])
    #     self.move_to_target_pose(des_pose)

    # def move_ym_cb(self, msg):
    #     rospy.loginfo("move_ym_cb")
    #     cur_pose = self.group.get_current_pose().pose
    #     des_pose = dxyz_wrt_eef_pose(cur_pose, [0, 0, DXYZ])
    #     self.move_to_target_pose(des_pose)

    # def move_zp_cb(self, msg):
    #     rospy.loginfo("move_zp_cb")
    #     cur_pose = self.group.get_current_pose().pose
    #     des_pose = dxyz_wrt_eef_pose(cur_pose, [0, DXYZ, 0])
    #     self.move_to_target_pose(des_pose)

    # def move_zm_cb(self, msg):
    #     rospy.loginfo("move_zm_cb")
    #     cur_pose = self.group.get_current_pose().pose
    #     des_pose = dxyz_wrt_eef_pose(cur_pose, [0, -DXYZ, 0])
    #     self.move_to_target_pose(des_pose)

    def approach_plan_cb(self, msg):
        self.group.set_joint_value_target(self.last_target_joint)
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
        '''
        if waypoints/update poses are updated,
        1) solve IK for eef pose of physical robot(real_eef_link) wrt /base_link
        2) sort based on the diff btw current joint
        3) publish lastly selected sol of robot states
            (if sol hasn't seleceted, publish clear state)
        '''
        if msg.poses:
            rospy.loginfo("waypoints_update_cb")
            try:
                (trans, rot) = self.listener.lookupTransform('/base_link', REAL_EEF_LINK, rospy.Time(0))
                rospy.loginfo("waypoints_update_cb| trans:{}, rot:{}".format(trans, rot))

                current_joint = self.group.get_current_joint_values()
                self.ur5_inv.solve_and_sort(trans, rot, current_joint)

                if self.last_sol_num is None: 
                    self.ur5_inv.publish_state(-1)
                else:
                    self.ur5_inv.publish_state(self.last_sol_num)

            except Exception as e:
                rospy.logerr(e)
                pass

    def record_move_cb(self, msg):
        for i in range(len(record_motion)):
            self.move_to_target_joint(record_motion[i])    
            rospy.loginfo("record_move_cb| joint {}".format(i))
            if i == 0:
                is_reset = pcl_fusion_reset()
                if is_reset == False:   break
        
        rospy.loginfo("record_move_cb| finished")

    def stitch_plan_cb(self, msg):
        self.move_to_target_joint(record_motion[self.count])
        rospy.loginfo("stitch_plan_cb| joint {}".format(self.count))

        if self.count == 0:    
            is_reset = pcl_fusion_reset()

        if (self.count > 0) or (is_reset == True):
            self.count += 1

    def distance_cb(self, msg):
        offset = msg.data

        rospy.loginfo("distance_cb| msg: {}".format(offset))
        self.last_offset = offset

    def solution_cb(self, msg):
        '''
        if sol num is clicked,
        publish selected sol(-1~7) of robot state and save it
        (if -1 is selected, save target joint as None)
        '''
        sol_num = msg.data

        rospy.loginfo("solution_cb| msg: {}".format(sol_num))
        
        _, target_joint = self.ur5_inv.publish_state(sol_num)

        if sol_num == -1:            
            self.last_target_joint = None
        else:
            self.last_target_joint = target_joint

        self.last_sol_num = sol_num

    def attach_sphere(self, link, name, pose, radius, touch_links):
        aco = moveit_msgs.msg.AttachedCollisionObject()
        aco.object = self.make_sphere(name, pose, radius) ##
        aco.link_name = link
        aco.touch_links = touch_links
        self.aco_pub.publish(aco)

    def make_sphere(self, name, pose, radius):
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.header = pose.header
        sphere = shape_msgs.msg.SolidPrimitive()
        sphere.type = shape_msgs.msg.SolidPrimitive.SPHERE
        sphere.dimensions = [radius]
        co.primitives = [sphere]
        co.primitive_poses = [pose.pose]
        return co

    def gripper_close_cb(self, msg):
        if msg.data:
            rospy.sleep(3)  # wait for close gripper
            attach_link = "left_inner_finger_pad"
            part_name = "part"
            part_size = 0.1 # sphere - radius
            # part_size = (0.12, 0.02, 0.15)    # box
            part_pose = geometry_msgs.msg.PoseStamped()
            part_pose.header.frame_id = "left_inner_finger_pad"
            part_pose.pose.position.y = -0.01
            part_pose.pose.position.z = 0.05
            touch_links = self.robot.get_link_names(group='robotiq')
            # self.scene.attach_box(link=attach_link, name=part_name, pose=part_pose, size=part_size, touch_links=touch_links)
            self.attach_sphere(attach_link, part_name, part_pose, part_size, touch_links)
            self.captured_pcl.publish(Bool(False))
        else:
            # dettach link
            pass


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