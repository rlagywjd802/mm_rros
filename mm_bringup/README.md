# mm_rros
These are packages for NIST mobile manipulator project.

version: **v5.2**
- final version for CASE paper
- tested on 04/29

# hardware
## actuator
- inspectorbot
- ur5
- robotiq gripper

## sensor
- kinect
- realsense
- webcam (x3)
- lidar
- imu

# software
## extra package list
- openni_launch
- realsense-ros(https://github.com/IntelRealSense/realsense-ros)
- usb_cam
- razor_imu_9dof(imu, raw value of imu)
- robot_pose_ekf(imu, sensor fusion)
- ur_modern_driver(arm, run robot)
- universal_robot(arm, meta package for ur)
- robotiq(gripper)
- robot_self_filter(https://github.com/rlagywjd802/robot_self_filter)
- PCLFusionColor

** missing package for lidar

# main
## how to run
- robot computer
roslaunch mm_bringup mm_demo_session_bringup.launch rviz:=false
rosrun mm_gui_run waypoint_controls.py
rosrun mm_gui_run mm_gui_moveit_approach_execution.py

- remote computer
roslaunch mm_bringup mm_demo_rviz.launch config:=true

## description
### fuse_points.launch(from REX's code)
stitch pointcloud
- input : pointcloud from realsense
- output : stitched pointcloud

### mm_gui_pcl_execution.py
visualize stitched pointcloud
- input : buttons under <Camera on Manipulator>, stitched pointcloud
- output : stitched pointcloud (for viz)

### waypoint_controls.py
creates ghost gripper according to user-setted position & parameters(rotation_axes, distance)
- input : clicked_point in rviz, buttons for 'set pre-grasp pose', 'displacement'
- output : ghost gripper interactive marker

### mm_gui_moveit_approach_execution.py
plans and executes the motion for ur5
- input : buttons under <Manipulator Motion>, buttons for select arm configuration 
- output : planned trajectory

# To-do
explain why commented out
add useful tutorials