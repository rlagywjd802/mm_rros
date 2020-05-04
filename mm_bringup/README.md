# mm_rros
These are packages for NIST mobile manipulator project.

version: **v5.2**
- final version for CASE paper
- tested on 04/30

## Hardware Components
### actuator
- inspectorbot
- ur5
- robotiq gripper

### sensor
- kinect
- realsense
- webcam (x3)
- lidar
- imu

## Installation
Before running the code, make sure that you have installed following packages.
Everything except for *PCLFusionColor* is open-source package.
- openni_launch(kinect)
- realsense-ros(realsense)
- usb_cam(webcam)
- razor_imu_9dof(imu) 
- robot_localization(imu)
- lms1xx(lidar)
- laser_filters(lidar)
- ur_modern_driver(ur5)
- universal_robot(ur5)
- robotiq(gripper)
- PCLFusionColor

## Usage
**Arm** Unfortunately, mobile base is broken. If you want to test out only arm part, you can try this.
On robot computer, run:
```roslaunch mm_bringup mm_demo_session_bringup.launch rviz:=false```
```rosrun mm_gui_run waypoint_controls.py```
```rosrun mm_gui_run mm_gui_moveit_approach_execution.py```
You can either directly launch above files from robot computer or launch via ssh on remote computer.
To access robot computer via ssh, run:
```ssh -X mm@mm```
On remote computer, run:
```roslaunch mm_bringup mm_demo_rviz.launch config:=true```

**Base+Arm** Full version for user trial. To run mobile base as well, uncomment following lines from *mm_demo_session_bringup.launch* and run same command as above.
```
<!-- <include file="$(find mm_bringup)/launch/mm_mobile_bringup.launch"/> -->
```
```
<!-- <include file="$(find mm_slam)/launch/mm_rtabmap.launch">
	<arg name="camera" 				value="kinect1"/>
	<arg name="localization"		value="true"/>
	<arg name="dwa"					value="false"/>
	<arg name="database_path" 		value="$(arg map_name)"/>
	<arg name="odom_topic"			value="/odom"/>
</include> -->
```

NOTE:
To map the area, change localization arg value from "true" to "false" and run same command.

## Description
*mm_demo_session_bringup.launch* is a main file for this package. It includes multiple launch files and nodes as follows.

### mm_mobile_bringup.launch
It includes *base_odom_publisher.py* from *mm_bringup* package. 
```
Arguments:
- device
```
Identify which USB port is being used for mobile base and set device value accordingly.

**mm_bringup/base_odom_publisher.py**
This is a ROS driver for inpectorbot(mobile base).
It provides follwing functionality by serial communication with mobile base controller(roboteq):
1. Computes and publishes nav_msgs/Odometry message by getting encoder value periodically.
2. Sends velocity commands to roboteq controller when it recieves geometry_msgs/Twist message.
```
Subscribed Topics:
- cmd_vel (geometry_msgs/Twist)
Published Topics:
- odom (nav_msgs/Odometry)
```

### mm_camera_bringup.launch
This launches kinect, realsense and webcams.

### mm_imu_bringup.launch
It includes *razor-pub.launch* from *razor_imu_9dof* package and *ekf_localization_node* from *robot_localization*.

**razor_imu_9dof/razor-pub.launch**
*razor_imu_9dof* is a ROS driver package for the Sparkfun Razor IMU 9DOF. You might need to calibrate the sensor for improving precision. Refer to http://wiki.ros.org/razor_imu_9dof for further information about calibration.
```
Published Topics:
- imu (sensor_msgs/Imu)
```

**robot_localization/ekf_localization_node**
*robot_localization* is a package for estimating the state of the robot based on different sensor sources. Extended Kalman Filter(EKF) is used to combine measurment from wheel odometry and imu sensor in *mm_imu_bringup.launch*. Refer to http://docs.ros.org/melodic/api/robot_localization/html/index.html for further information.
```
Subscribed Topics:
- odom (nav_msgs/Odometry)
- imu (sensor_msgs/Imu)
Published Topics:
- odom_filtered (nav_msgs/Odometry)
```

### mm_laser_bringup.launch
```
Arguments:
- host
```
Identify IP address of the lidar and set host value accordingly.

**lms1xx/LMS1xx_node**
*lms1xx* is a ROS driver package for SICK lms1xx lidar.
```
Published Topics:
- scan (sensor_msgs/LaserScan)
```

**laser_filters/scan_to_scan_filter_chain**
*laser_filters* package provides number of general purpose filters for sensor_msgs/LaserScan messages. LaserScanBoxFilter is used in *mm_laser_bringup.launch*. You can either set corresponding range values from * mm_bringup/params/my_laser_config.yaml* or use differnt filter plugin. Refer to http://wiki.ros.org/laser_filters for further information about filter plugins. 
```
Subscribed Topics:
- scan (sensor_msgs/LaserScan)
Published Topics:
- scan_filtered (sensor_msgs/LaserScan)
```

### mm_ur5_driver.launch
```
Arguments:
- robot_ip
```
Identify IP address of the universal robot controller and set robot_ip value accordingly.

**ur_modern_driver/ur_driver**
*ur_moden_driver* is a ROS driver package for universal robots. Refer to https://github.com/ros-industrial/ur_modern_driver for further information.
```
Published Topics:
- ur5/joint_states (sensor_msgs/JointState)
Actions:
- follow_joint_trajectory
```

### mm_gripper_bringup.launch
```
Arguments:
- device
```
Identify which USB port is being used for gripper and set device value accordingly.

**robotiq_2f_gripper_control/robotiq_2f_gripper_ctrl_gui.py**
This is a ROS wrapper for robotiq driver. If gripper_close value is True, it will close the gripper. Else, it will open the gripper.
```
Subscribed Topics:
- gripper_close (std_msgs/Bool)
Published Topics:
- robotiq/joint_states (sensor_msgs/JointState)
```

### description.launch
Set robot_description parameter as *mm_description/mm.urdf.xacro*.

### mm_rtabmap.launch
```
Arguments:
- camera : 3D camera name used for SLAM & Localization
- localization : If it is True, localization and navigation will work based on saved map database.
- dwa : If it is True, use Dynamic Window Approach(dwa) as local planner. Else, use Time Elastic Band(teb) as local planner
- database_path : relative path for mapped database
- odom_topic : If you are not using imu, set this as /odom. If you are using imu, set this as /odometry/filtered. 
```

*mm_navigation/mm_move_base_teb.launch*
This lets you move a robot to desired positions using the navigation stack. Refer to http://wiki.ros.org/move_base for further parameters.

*rtabmap*
RTAB-Map(Real-Time Appearance-Based Mapping) package provides RGBD based SLAM approach. There is multiple RTAB-Map configurations that can be used for your robot. Kinect+Odometry is used for *mm_rtabmap.launch*. Refer to http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot for further information about configurations and parameters.

### fuse_points.launch
*point_cloud_stitching_color* package stitches a sequence of pointclouds to reconstruct a 3D scene using Iterative Closest Point (ICP) algorithm. Everything in PCLFusionColor folder is from REX.
```
Arguments:
- fusion_frame : reference frame for stitching
- cloud_in : topic name for input pointcloud data
```
**point_cloud_stitching_color/pcl_stitch_node_color**
cloud_in value is remapped as input_point_cloud.
```
Subscribed Topics:
- input_point_cloud (sensor_msgs/PointCloud2)
Published Topics:
- pcl_fusion_node/fused_points (sensor_msgs/PointCloud2)
```

### mm_gui_pcl_execution.py
This is used for showing or clearing stitched pointcloud data in rviz only when the user clicks button.
```
Subscribed Topics:
- pcl_capture (std_msgs/Bool)
- pcl_clear (std_msgs/Bool)
- captured_cloud_in (sensor_msgs/PointCloud2)
Published Topics:
- captured_pcl (sensor_msgs/PointCloud2)
```

### waypoint_controls.py
This node creates gripper-shaped interactive marker according to the clicked point and parameters(rotation_axes, distance) in gui.
```
Subscribed Topics:
- clicked_point (PointStamped)
- rotate_axis (std_msgs/String)
- distance (std_msgs/Int32)
Published Topics:
- waypoints/updates (visualization_msgs/InteractiveMarkerUpdate)
```

### mm_gui_moveit_approach_execution.py
This is used for solving ik from gripper marker pose, plannning the motion to the joint target and executing the plan. It also provides plans for manual motion and execution.
```
Subscribed Topics:
- waypoints/updates (visualization_msgs/InteractiveMarkerUpdate)
```