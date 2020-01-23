#!/usr/bin/env python
import rospy
import sys

import tf
import math

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

from utils import *

FRAME_ID = "base_footprint"
GRIPPER_MESH = "package://mm_description/meshes/gripper/robotiq_2f85_opened_combined_axis_mated.STL"
PRE_OFFSET = 0.15
QUAT_DOWNWARD = tf.transformations.quaternion_from_euler(0, math.pi/2.0, -math.pi/2.0)
QUAT_FORWARD = tf.transformations.quaternion_from_euler(0, 0, -math.pi/2.0)

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

        # subscriber
        rospy.Subscriber("clicked_point", PointStamped, self.clicked_cb)
 
        rospy.Subscriber("add_waypoint", Bool, self.add_waypoint_cb)
        rospy.Subscriber("remove_waypoint", Bool, self.remove_waypoint_cb)

        # rospy.Subscriber("change_direction", Bool, self.change_direction_cb)
        rospy.Subscriber("approach_forward", Bool, self.approach_forward_cb)

        # publisher
        self.update_rate = rospy.Rate(2)

        # create an interactive marker server on the topic namespace waypoint
        self.server = InteractiveMarkerServer("waypoints")

        # interactive markers
        self.direction = 'forward'    # forward or downward
        self.last_poses = []
        self.num_of_wpt = 0

        # clicked point
        self.clicked_pose = None

    ##################################################################################################
    ##################################################################################################

    def process_feedback(self, feedback):
        f_name = feedback.marker_name
        f_pose = feedback.pose

        rospy.logdebug("process_feedback| {} is now at ".format(f_name) + print_pose(f_pose, ""))

        self.last_poses[int(f_name)] = f_pose
        self.server.applyChanges()

    def update_imarker(self):
        if self.num_of_wpt <= 0:
            rospy.logerr("update_imarker| num_of_wpt is {}".format(self.num_of_wpt))
            return

        # add last feedback pose in update msg
        int_marker_poses = []
        for i in range(self.num_of_wpt):
            imp = InteractiveMarkerPose()
            imp.pose = self.last_poses[i]
            imp.header.frame_id = FRAME_ID
            imp.name = str(i)
            int_marker_poses.append(imp)

        int_marker_update = InteractiveMarkerUpdate()
        int_marker_update.server_id = 'waypoints_gui_control'
        int_marker_update.poses = int_marker_poses
        
        self.server.publish(int_marker_update)
        self.server.applyChanges()

    def make_mesh_marker(self, offset=0):
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

        marker.pose.position.x = -offset
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

    def insert_gripper(self, num, gripper_point):         
        # create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = FRAME_ID
        int_marker.name = str(num)
        int_marker.description = "wpt "+int_marker.name
        int_marker.scale = .1

        # create a non-interactive control which contains the box
        gripper_control = InteractiveMarkerControl()
        gripper_control.always_visible = True

        gripper_marker = self.make_mesh_marker()
        gripper_control.markers.append( gripper_marker )
        
        if num == 0:
            gripper_marker_pre = self.make_mesh_marker(PRE_OFFSET)
            gripper_control.markers.append( gripper_marker_pre )

        # add the control to the interactive marker
        int_marker.controls.append( gripper_control )
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        # create a control which will move the box
        rotate_control = self.make_imarker_ctrl("move", "x")
        int_marker.controls.append(rotate_control)

        rotate_control = self.make_imarker_ctrl("move", "y")
        int_marker.controls.append(rotate_control)

        rotate_control = self.make_imarker_ctrl("move", "z")
        int_marker.controls.append(rotate_control)

        if num == 0:
            if self.direction is 'downward':
                rotate_control = self.make_imarker_ctrl("rotate", "x")
                int_marker.controls.append(rotate_control)
            else:
                rotate_control = self.make_imarker_ctrl("rotate", "z")
                int_marker.controls.append(rotate_control)

        # add the interactive marker tolast_poses our collection
        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

        # set gripper orientation based on direction
        if self.direction is 'downward':
            q = QUAT_DOWNWARD
        else:
            q = QUAT_FORWARD

        # set gripper pose
        gripper_pose = Pose()
        gripper_pose.position = gripper_point
        gripper_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self.server.setPose( int_marker.name, gripper_pose )
        self.server.applyChanges()

        rospy.logdebug('insert_gripper| dir: {}, '.format(self.direction) + print_pose(gripper_pose))

        return gripper_pose

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
        # add first waypoint to the server
        inserted_pose = self.insert_gripper(0, clicked_point)

        # update interactive marker information
        self.last_poses = [inserted_pose]
        self.num_of_wpt = 1

        self.clicked_pose = inserted_pose
      
    def add_waypoint_cb(self, msg):
        rospy.loginfo("add_waypoint_cb| {}".format(msg.data))

        # data is True if add button is clicked
        if msg.data:
            # user can add waypoint only after grasping point is clicked
            if self.num_of_wpt > 0:
                # set initial position of the interactive marker
                initial_point = Point()
                initial_point.x = self.clicked_pose.position.x
                initial_point.y = self.clicked_pose.position.y - 1.0
                initial_point.z = self.clicked_pose.position.z
                
                # add one waypoint to the server
                inserted_pose = self.insert_gripper(self.num_of_wpt, initial_point)

                # update interactive marker information
                self.last_poses.append(inserted_pose)
                self.num_of_wpt += 1
            else:
                rospy.logerr("add_waypoint_cb| num_of_wpt is {}".format(self.num_of_wpt))
    
    def remove_waypoint_cb(self, msg):
        rospy.loginfo("remove_waypoint_cb|")

        # data is True if remove button is clicked
        if msg.data:
            if self.num_of_wpt <= 0:
                rospy.logerr("remove_waypoint_cb| num_of_wpt is {}".format(self.num_of_wpt))
                return

            # remove last waypoint to the server
            self.erase_waypoint()

            # update interactive marker information
            self.last_poses.pop()
            self.num_of_wpt -= 1

    def change_direction_cb(self, msg):
        rospy.loginfo("change_direction_cb|")

        # data is True if remove button is clicked
        if msg.data:
            if self.num_of_wpt != 1:
                rospy.logerr("change_direction_cb| num_of_wpt is {}".format(self.num_of_wpt))
                return

            # remove last waypoint to the server
            self.erase_waypoint()

            # flip direction
            if self.direction is 'forward':
                self.direction = 'downward'
            else:
                self.direction = 'forward'

            inserted_pose = self.insert_gripper(0, self.last_poses[0].position)

            # update interactive marker information
            self.last_poses.pop()
            self.last_poses.append(inserted_pose)

    def approach_forward_cb(self, msg):
        rospy.loginfo("approach_forward_cb|")

        # direction can only be changed when there is no additional waypoints
        if self.num_of_wpt != 1:
            rospy.logerr("change_direction_cb| num_of_wpt is {}".format(self.num_of_wpt))
            return

        if msg.data:
            command = 'forward'
        else:
            command = 'downward'

        if command is not self.direction:
            self.direction = command

            # remove last waypoint to the server
            self.erase_waypoint()

            inserted_pose = self.insert_gripper(0, self.last_poses[0].position)

            # update interactive marker information
            self.last_poses.pop()
            self.last_poses.append(inserted_pose)

def main(arg):
    if len(arg) > 1:
        if arg[1] == "debug":
            log_level = rospy.DEBUG
    else:
        log_level = rospy.INFO

    try:
        wgc = WaypointsGUIControl(log_level)

        while not rospy.is_shutdown():
            if wgc.num_of_wpt > 0:
                wgc.update_imarker()
            wgc.update_rate.sleep()

    except KeyboardInterrupt:
        return

if __name__== "__main__":
    main(sys.argv)