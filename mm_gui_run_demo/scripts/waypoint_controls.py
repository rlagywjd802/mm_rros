#!/usr/bin/env python
import rospy
import sys

import tf
import math

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

FRAME_ID = "base_footprint"
# GRASP_ORIEN_X = 0.514690293398
# GRASP_ORIEN_Y = 0.492745545107
# GRASP_ORIEN_Z = -0.510770083592
# GRASP_ORIEN_W = 0.481050570488
GRASP_ORIEN_X = 0.0
GRASP_ORIEN_Y = 0.0
GRASP_ORIEN_Z = 0.0
GRASP_ORIEN_W = 0.0


def normalizeQuaternion( quaternion_msg ):
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

        # publisher
        self.update_rate = rospy.Rate(2)

        # create an interactive marker server on the topic namespace waypoint
        self.server = InteractiveMarkerServer("waypoints")

        # interactive markers
        self.last_poses = []
        self.num_of_wpt = 0

        # clicked point
        self.clicked_pose = None

    def process_feedback(self, feedback):
        p = feedback.pose.position
        rospy.logdebug("process_feedback| {} is now at {}, {}, {}".format(feedback.marker_name, p.x, p.y, p.z))

        self.last_poses[int(feedback.marker_name)] = feedback.pose
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

    def make_box(self, scale):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def make_sphere(self, scale):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def make_mesh(self, offset=0):
        marker = Marker()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://mm_description/meshes/gripper/robotiq_arg2f_85_combined_opened_t.STL";
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        # marker.pose.position.x = -0.13
        # marker.pose.position.y = 0.15/2. + 0.004
        # marker.pose.position.z = -0.07/2. - 0.001
        # marker.pose.position.x = -0.07/2. - 0.001
        # marker.pose.position.y = -0.15/2. - 0.004
        # marker.pose.position.z = -0.13 - offset
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = -0.13 -offset
        quat = tf.transformations.quaternion_from_euler(math.pi/2, 0, math.pi/2)

        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        return marker

    def insert_waypoint(self, num, initial_pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = FRAME_ID
        int_marker.name = str(num)
        int_marker.description = "wpt "+int_marker.name
        int_marker.scale = .1

        # create a box marker
        box_marker = self.make_sphere(.03);

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )

        # add the control to the interactive marker
        int_marker.controls.append( box_control )
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_3D

        # create a control which will move the box
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 1
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 0
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_y"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 1
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_z"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 1
        rotate_control.orientation.z = 0
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        # add the interactive marker tolast_poses our collection
        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

        self.server.setPose( int_marker.name, initial_pose )
        self.server.applyChanges()

    def insert_waypoint_gripper(self, num, initial_pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = FRAME_ID
        int_marker.name = str(num)
        int_marker.description = "wpt "+int_marker.name
        int_marker.scale = .1

        gripper_maker = self.make_mesh();

        # create a non-interactive control which contains the box
        gripper_control = InteractiveMarkerControl()
        gripper_control.always_visible = True
        # gripper_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        gripper_control.markers.append( gripper_maker )

        # add the control to the interactive marker
        int_marker.controls.append( gripper_control )
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        # create a control which will move the box
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_x"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 1
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 0
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate_control);

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 1
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 0
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_y"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 1
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_z"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 1
        rotate_control.orientation.z = 0
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        # add the interactive marker tolast_poses our collection
        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

        self.server.setPose( int_marker.name, initial_pose )
        self.server.applyChanges()

    def insert_gripper(self, num, initial_pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = FRAME_ID
        int_marker.name = str(num)
        int_marker.description = "wpt "+int_marker.name
        int_marker.scale = .1

        gripper_maker = self.make_mesh();
        gripper_maker_pre = self.make_mesh(0.15);

        # create a non-interactive control which contains the box
        gripper_control = InteractiveMarkerControl()
        gripper_control.always_visible = True
        # gripper_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        gripper_control.markers.append( gripper_maker )
        gripper_control.markers.append( gripper_maker_pre )

        # add the control to the interactive marker
        int_marker.controls.append( gripper_control )
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        # create a control which will move the box
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_x"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 1
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 0
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate_control);

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 1
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 0
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_y"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 0
        rotate_control.orientation.z = 1
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_z"
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 1
        rotate_control.orientation.z = 0
        normalizeQuaternion(rotate_control.orientation)
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        # add the interactive marker tolast_poses our collection
        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

        self.server.setPose( int_marker.name, initial_pose )
        self.server.applyChanges()


    def erase_waypoint(self):
        # remove the interactive marker from server
        self.server.erase(str(self.num_of_wpt-1))
        self.server.applyChanges()

    def clicked_cb(self, msg):
        rospy.loginfo("clicked [x, y, z]: "+str(msg.point.x)+","+str(msg.point.y)+","+str(msg.point.z))

        # set initial position of the interactive marker as clicked point
        initial_pose = Pose()
        initial_pose.position = msg.point

        quat = tf.transformations.quaternion_from_euler(0, math.pi/2.0, -math.pi/2.0)
        initial_pose.orientation.x = quat[0]
        initial_pose.orientation.y = quat[1]
        initial_pose.orientation.z = quat[2]
        initial_pose.orientation.w = quat[3]
        # add one waypoint to the server
        self.insert_gripper(str(0), initial_pose)

        # update interactive marker information
        self.last_poses = [initial_pose]
        self.num_of_wpt = 1

        self.clicked_pose = initial_pose

        
    def add_waypoint_cb(self, msg):
        rospy.loginfo("add_waypoint_cb| {}".format(msg.data))

        # data is True if add button is clicked
        if msg.data:
            # user can add waypoint only after grasping point is clicked
            if self.num_of_wpt > 0:
                # set initial position of the interactive marker
                initial_pose = Pose()
                initial_pose.position.x = self.clicked_pose.position.x
                initial_pose.position.y = self.clicked_pose.position.y + 0.3
                initial_pose.position.z = self.clicked_pose.position.z
                quat = tf.transformations.quaternion_from_euler(0, math.pi/2.0, -math.pi/2.0)
                initial_pose.orientation.x = quat[0]
                initial_pose.orientation.y = quat[1]
                initial_pose.orientation.z = quat[2]
                initial_pose.orientation.w = quat[3]
                
                # add one waypoint to the server
                self.insert_waypoint_gripper(self.num_of_wpt, initial_pose)

                # update interactive marker information
                self.last_poses.append(initial_pose)
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