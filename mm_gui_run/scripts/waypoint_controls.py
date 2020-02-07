#!/usr/bin/env python
import rospy
import sys

import copy
import math

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

from utils import *
from const import *

def normalizeQuaternion(quaternion_msg):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s

class WaypointsGUIControl():
    def __init__(self, log_level):
        rospy.init_node("waypoints_gui_control", log_level=log_level)

        # Subscriber
        rospy.Subscriber("clicked_point", PointStamped, self.clicked_cb)

        rospy.Subscriber("rotate_axis", String, self.rotate_axis_cb)
        rospy.Subscriber("distance", Int32, self.distance_cb)

        rospy.Subscriber("clear_imarker", Bool, self.clear_imarker_cb)

        # Pubisher
        self.target_pose_br = tf.TransformBroadcaster()
        self.update_rate = rospy.Rate(4)

        self.instruction_pub = rospy.Publisher('/instruction', String, queue_size=1)
        self.is_published = False

        # create an interactive marker server on the topic namespace waypoint
        self.server = InteractiveMarkerServer("waypoints")

        # interactive markers
        self.last_poses = []
        self.num_of_wpt = 0

        # clicked point
        self.clicked_pose = None

        self.last_r_axis = copy.deepcopy(INIT_R_AXIS)
        self.last_offset = copy.deepcopy(INIT_OFFSET)

    ##################################################################################################
    ##################################################################################################

    def process_feedback(self, feedback):
        f_name = feedback.marker_name
        f_pose = feedback.pose

        # update marker pose when mouse button is released 
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo("process_feedback| {} is now at ".format(f_name) + print_pose(f_pose, ""))

            self.update_tf(f_pose)
            self.last_poses[int(f_name)] = f_pose
            self.server.applyChanges()

    # def update_imarker(self):
    #     if self.num_of_wpt <= 0:
    #         rospy.logerr("update_imarker| num_of_wpt is {}".format(self.num_of_wpt))
    #         return

    #     # add last feedback pose in update msg
    #     int_marker_poses = []
    #     for i in range(self.num_of_wpt):
    #         imp = InteractiveMarkerPose()
    #         imp.pose = self.last_poses[i]
    #         imp.header.frame_id = FRAME_ID
    #         imp.name = str(i)
    #         int_marker_poses.append(imp)

    #     int_marker_update = InteractiveMarkerUpdate()
    #     int_marker_update.server_id = 'waypoints_gui_control'
    #     int_marker_update.poses = int_marker_poses
        
    #     self.server.publish(int_marker_update)
    #     self.server.applyChanges()

    #     self.update_tf(int_marker_poses[0].pose, self.last_offset)

    def update_tf(self, pose=None, offset=None):
        '''
        update tf for real_eef_pose
        '''
        if pose is None:
            pose = self.last_poses[0]
        if offset is None:
            offset = self.last_offset

        time_now = rospy.Time.now()
        pp = pose.position
        po = pose.orientation
        offset = offset/100.0
        
        # FRAME_ID --> marker_pose
        self.target_pose_br.sendTransform((pp.x, pp.y, pp.z), 
                            (po.x, po.y, po.z, po.w), 
                            time_now, 
                            "marker_pose", 
                            FRAME_ID)

        # marker_pose --> gripper_pose
        self.target_pose_br.sendTransform((-offset, 0, 0), 
                            (0.0, 0.0, 0.0, 1.0), 
                            time_now, 
                            "gripper_pose", 
                            "marker_pose")

        # gripper_pose --> eef_pose
        quat = euler_to_quat(math.pi/2, 0, 0)
        self.target_pose_br.sendTransform((-0.13, 0, 0), 
                    (quat.x, quat.y, quat.z, quat.w), 
                    time_now, 
                    "eef_pose", 
                    "gripper_pose")

        # eef_pose --> real_eef_pose
        quat = euler_to_quat(-math.pi/2, 0, -math.pi/2)
        self.target_pose_br.sendTransform((0, 0, 0), 
                    (quat.x, quat.y, quat.z, quat.w), 
                    time_now, 
                    "real_eef_pose", 
                    "eef_pose")

    def make_mesh_marker(self, offset=0.0):
        marker = Marker()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = GRIPPER_MESH
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker.pose.position.x = -offset/100.0 # cm to m
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        return marker

    def make_imarker_ctrl(self, ctrl_mode, ctrl_axis):
        imarker_ctrl = InteractiveMarkerControl()
        imarker_ctrl.name = ctrl_mode+'_'+ctrl_axis

        # set control orientation
        imarker_ctrl.orientation.w = 1
        if ctrl_axis is 'x':
            imarker_ctrl.orientation.x = 1
        elif ctrl_axis is 'y':
            imarker_ctrl.orientation.z = 1
        elif ctrl_axis is 'z':
            imarker_ctrl.orientation.y = 1
        else:
            rospy.logerr('make_imarker_ctrl| wrong control axis')
        normalizeQuaternion(imarker_ctrl.orientation)

        # set control mode
        if ctrl_mode is "move":
            imarker_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        elif ctrl_mode is "rotate":
            imarker_ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            rospy.logerr('make_imarker_ctrl| wrong control mode')

        return imarker_ctrl

    def insert_gripper(self, num, pose, r_axis, offset):
        log_msg = 'insert_gripper| num: {}'.format(num)
        log_msg += ', r_axis:' + r_axis
        log_msg += ', offset: {}'.format(offset)
        log_msg += print_pose(pose, ', pose:')
        rospy.loginfo(log_msg)         
        
        # create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = FRAME_ID
        int_marker.name = str(num)
        int_marker.description = "wpt "+int_marker.name
        int_marker.scale = .1

        # create a non-interactive control which contains the box
        gripper_control = InteractiveMarkerControl()
        gripper_control.always_visible = True
        
        if num == 0:
            gripper_marker_pre = self.make_mesh_marker(offset)
            gripper_control.markers.append( gripper_marker_pre )
        else:
            gripper_marker = self.make_mesh_marker()
            gripper_control.markers.append( gripper_marker )

        # add the control to the interactive marker
        int_marker.controls.append( gripper_control )
        # int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.ROTATE_3D

        if num == 0:
            rotate_control = self.make_imarker_ctrl("rotate", r_axis)
            int_marker.controls.append(rotate_control)

        # update tf
        self.update_tf(pose, offset)
        
        # add the interactive marker to last_poses our collection
        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

        # set gripper pose
        self.server.setPose(int_marker.name, pose)
        self.server.applyChanges()

    def erase_waypoint(self):
        # remove the interactive marker from server
        self.server.erase(str(self.num_of_wpt-1))
        self.server.applyChanges()

    ##################################################################################################
    ##################################################################################################

    def clicked_cb(self, msg):
        clicked_point = msg.point        
        rospy.loginfo("clicked_cb| clicked "+print_point(clicked_point))

        # set initial point of the interactive marker as clicked point
        initial_pose = Pose()
        initial_pose.position = clicked_point
        initial_pose.orientation = INIT_ORIENT

        # add first waypoint to the server
        self.insert_gripper(0, initial_pose, self.last_r_axis, self.last_offset)

        # update interactive marker information
        self.last_poses = [initial_pose]
        self.num_of_wpt = 1

        self.clicked_pose = initial_pose

        self.instruction_pub.publish(STEP3)

    def rotate_axis_cb(self, msg):
        rotate_axis = msg.data

        rospy.loginfo("rotate_axis_cb| r_axis: {}, offset: {}".format(rotate_axis, self.last_offset))

        self.erase_waypoint()
        self.insert_gripper(0, self.last_poses[0], rotate_axis, self.last_offset)

        self.last_r_axis = rotate_axis

    def distance_cb(self, msg):
        offset = msg.data
        rospy.loginfo("distance_cb| r_axis: {}, offset: {}".format(self.last_r_axis, offset))

        self.erase_waypoint()
        self.insert_gripper(0, self.last_poses[0], self.last_r_axis, offset)

        self.last_offset = offset

    def clear_imarker_cb(self, msg):
        if msg.data:
            self.erase_waypoint()            
            self.last_r_axis = INIT_R_AXIS
            self.last_offset = INIT_OFFSET
            self.last_poses[0] = self.clicked_pose
            rospy.loginfo("clear_imarker_cb| r_axis:{}, offset:{}, ".format(self.last_r_axis, self.last_offset)+print_pose(self.last_poses[0]))
            self.insert_gripper(0, self.last_poses[0], self.last_r_axis, self.last_offset)

    def publish_step1(self):
        if not self.is_published:
            self.instruction_pub.publish(STEP1)
            self.is_published = True

def main(arg):
    if len(arg) > 1:
        if arg[1] == "debug":
            log_level = rospy.DEBUG
    else:
        log_level = rospy.INFO

    try:
        wgc = WaypointsGUIControl(log_level)
        count = 0
        
        while not rospy.is_shutdown():
            if(count > 10):
                wgc.publish_step1()
            # if wgc.num_of_wpt > 0:
            #     wgc.update_tf()
            count += 1
            wgc.update_rate.sleep()
    except KeyboardInterrupt:
        return

if __name__== "__main__":
    main(sys.argv)