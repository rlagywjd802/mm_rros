#!/usr/bin/env python
import sys
import copy
import rospy
import tf
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import shape_msgs.msg
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose, PointStamped, PoseStamped
from visualization_msgs.msg import InteractiveMarkerUpdate
from sensor_msgs.msg import Joy

from std_srvs.srv import Trigger, Empty

from ur5_inv_kin_wrapper import ur5_inv_kin_wrapper
from utils import *
from const import *

DXYZ = 0.02

# fixed motion for record
initial_joint = [-158.57, -61.82, -100.14, -95.34, 72.82, -247.12]
initial_joint = [j*math.pi/180.0 for j in initial_joint]
joint0 = [-2.65160733858, -0.787968460714, -1.77485400835, -1.28218108812, 1.23793566227, -4.4600678126]
joint1 = [-3.13806707064, -0.875796620046, -1.76907188097, -1.15789491335, 1.61859285831, -4.76847023169]
joint2 = [-3.51146394411, -0.984417740499, -1.73061067263, -1.13680297533, 1.90909790993, -5.00915128389]
joint3 = [-3.96667200724, -1.25426799456, -1.56269914309, -1.21947080294, 2.22984671593, -5.38175660769]
joint4 = [-4.133831803, -1.46375877062, -1.37870961825, -1.31245834032, 2.32692122459, -5.56171733538]
joint5 = [-4.28966409365, -1.84895164171, -0.921948734914, -1.52604610125, 2.39951276779, -5.75916833082]
record_motion = [joint0, joint1, joint2, joint3, joint4, joint5]

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

        rospy.Subscriber("pick_approach_plan", Bool, self.pick_approach_plan_cb)
        rospy.Subscriber("pick_approach_execute", Bool, self.pick_approach_execute_cb)
        rospy.Subscriber("pick_retreat_plan", Bool, self.pick_retreat_plan_cb)
        rospy.Subscriber("pick_retreat_execute", Bool, self.pick_retreat_execute_cb)
        rospy.Subscriber("place_approach_plan", Bool, self.place_approach_plan_cb)
        rospy.Subscriber("place_approach_execute", Bool, self.place_approach_execute_cb)
        rospy.Subscriber("place_retreat_plan", Bool, self.place_retreat_plan_cb)
        rospy.Subscriber("place_retreat_execute", Bool, self.place_retreat_execute_cb)

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

        self.pick_approach_plan = None
        self.pick_retreat_plan = None
        self.pick_retreat_plan1 = None
        self.place_approach_plan = None
        self.place_retreat_plan = None

        # set after execution
        self.pre_grasp_joint = None
        self.post_grasp_joint = None
        self.initial_joint = joint3

        self.pick_retreat_step = 0

        self.last_offset = 10.0

        # IK
        self.ur5_inv = ur5_inv_kin_wrapper()
        self.last_selected_joint = None
        self.last_sol_num = None

        self.count = 0

        self.move_to_target_joint(initial_joint)
        rospy.loginfo("move to initial joint")

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

    def plan_for_joint_target(self, joint):
        rospy.loginfo("plan to joint target| started")
        self.group.set_joint_value_target(joint)
        plan = self.group.plan()
        self.group.clear_pose_targets()
        rospy.loginfo("plan to joint target| finished")
        return plan

    def execute_plan(self, plan):
        rospy.loginfo("execute plan| started")
        self.group.execute(plan, wait=False)
        rospy.sleep(5)
        rospy.loginfo("execute plan| finished")

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

    ##################################################################################################
    ##################################################################################################

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

    def pick_approach_plan_cb(self, msg):
        '''
        initial joint -> pre-grasp joint
        '''
        if self.last_selected_joint is not None:
            self.pick_approach_plan = self.plan_for_joint_target(self.last_selected_joint)

    def pick_approach_execute_cb(self, msg):
        if self.pick_approach_plan is not None:
            self.execute_plan(self.pick_approach_plan)
            self.pre_grasp_joint = self.group.get_current_joint_values()
            rospy.loginfo("pick_approach_execute", self.pre_grasp_joint)
            self.instruction_pub.publish(STEP4)

    def pick_retreat_plan_cb(self, msg):
        '''
        grasp joint -> pre-grasp joint -> initial joint
        '''
        if self.pre_grasp_joint is not None:
            self.ur5_inv.publish_state(-1)

            self.pick_retreat_plan = self.plan_for_joint_target(initial_joint)
            # if self.pick_retreat_step == 0:
            #     self.pick_retreat_plan = self.plan_for_joint_target(self.pre_grasp_joint)
            #     rospy.loginfo("pick retreat plan | step 0")
            # elif self.pick_retreat_step == 1:
            #     self.pick_retreat_plan1 = self.plan_for_joint_target(initial_joint)
            #     rospy.loginfo("pick retreat plan | step 1")

    def pick_retreat_execute_cb(self, msg):
        if self.pick_retreat_plan is not None:
            self.execute_plan(self.pick_retreat_plan)
            # if self.pick_retreat_step == 0:
            #     self.execute_plan(self.pick_retreat_plan)
            #     self.pick_retreat_step += 1
            # elif self.pick_retreat_step == 1:
            #     self.execute_plan(self.pick_retreat_plan1)
            #     self.pick_retreat_step += 1

    def place_approach_plan_cb(self, msg):
        pass

    def place_approach_execute_cb(self, msg):
        pass

    def place_retreat_plan_cb(self, msg):
        pass

    def place_retreat_execute_cb(self, msg):
        pass

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
                elif self.last_sol_num is -1:
                    self.ur5_inv.publish_state(-1)
                    self.last_selected_joint = None
                else:
                    _, target_joint = self.ur5_inv.publish_state(self.last_sol_num)
                    self.last_selected_joint = target_joint

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
            self.last_selected_joint = None
        else:
            self.last_selected_joint = target_joint

        self.last_sol_num = sol_num

    def gripper_close_cb(self, msg):
        if msg.data:
            rospy.sleep(2)  # wait for close gripper
            attach_link = "left_inner_finger_pad"
            part_name = "part"
            part_size = 0.1 # sphere - radius
            # part_size = (0.12, 0.02, 0.15)    # box
            part_pose = geometry_msgs.msg.PoseStamped()
            part_pose.header.frame_id = "left_inner_finger_pad"
            part_pose.header.stamp = rospy.Time.now()
            part_pose.pose.position.y = -0.01
            part_pose.pose.position.z = 0.05
            touch_links = self.robot.get_link_names(group='robotiq')
            # self.scene.attach_box(link=attach_link, name=part_name, pose=part_pose, size=part_size, touch_links=touch_links)
            self.attach_sphere(attach_link, part_name, part_pose, part_size, touch_links)
            self.captured_pcl.publish(Bool(False))
        else:
            self.scene.remove_attached_object("left_inner_finger_pad", "part")
            rospy.sleep(1)
            self.scene.remove_world_object("part")
            self.captured_pcl.publish(Bool(False))

def main(arg):
    if len(arg) > 1:
        if arg[1] == "debug":
            log_level = rospy.DEBUG
    else:
        log_level = rospy.INFO

    cnt = 0 # publish transparent robot state for few times
    try:
        ur5 = UR5MoveGroupGUI(log_level)
        while not rospy.is_shutdown():
            if cnt < 2:
                ur5.ur5_inv.publish_state(-1)
            cnt += 1
            rospy.sleep(1)

    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main(sys.argv)