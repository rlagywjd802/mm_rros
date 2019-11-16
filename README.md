# basic commands
## ssh
ssh -X mm@192.168.0.111

## visualize the robot model
roslaunch mm_description display.launch

## publish joint and robot state of the robot
roslaunch mm_bringup mm_state_publisher.launch

## keyboard command
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

## rgbd remote visualize
roslaunch mm_slam rgbd_remote_viz.launch camera:=kinect1

## bringup the robot (default)
### base
rosrun mm_bringup base_odom_publisher.py
### multiple kinect
roslaunch openni_launch openni.launch device_id:=#1 camera:=kinect1 depth_registration:=true
roslaunch openni_launch openni.launch device_id:=#2 camera:=kinect2 depth_registration:=true
### arm (MoveIt!)
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.9
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch 
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
-- robot_description from ur5_upload.launch(ur5_brinup.launch)
-- robot_state_publisher from ur_common.launch(ur5_bringup.launch)
### gripper
roslaunch robotiq_2f_85_gripper_visualization test_2f_85_model.launch
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSBgripper
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py
### imu
roslaunch razor_imu_9dof razor-pub.launch
roslaunch robot_pose_ekf.launch vo_used:=false

## move the whole robot (compressed version)
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSBbase
roslaunch mm_bringup mm_kinect_bringup.launch camera1:=kinect1 camera2:=kinect2 rgbd:=true
roslaunch mm_bringup mm_ur5_bringup.launch
roslaunch mm_bringup mm_gripper_bringup.launch device:=/dev/ttyUSBgripper

# changes in packages
## base odom publisher
- changed v_max=0.2 and w_max=0.4
- delete test_pose
- add odom_test_pose
- add device

## ur_moder_driver
- add mm_ur5_bringup.launch to /launch

## razor_imu_9dof
- deleted remapping odom to /pr2~/odom
- changed subscribed topic from imu_data to imu

## robotiq
- 

## robot_pose_ekf
- 


# test modes

## base - slam record - keyboard
### robot
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSBbase
roslaunch mm_bringup mm_kinect_bringup.launch camera1:=true rgbd1:=true
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 mm:=false database_path:=<file_name>

### remote pc
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
roslaunch mm_slam base_rtabmap_rviz.launch rgbd1:=true
rosbag record /odom /tf /tf_static /joint_states /kinect1/depth_registered/image_raw /kinect1/depth_registered/camera_info /kinect1/rgb/image_rect_color /kinect1/rgb/camera_info /kinect_scan
rqt_console

## base - slam playback
rosbag play "<file_name>.bag"
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 mm:=false rviz:=true
rosrun map_server map_saver map:=/rtabmap/proj_map
** tf error

## base - localization - move_base
### robot
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSBbase
roslaunch mm_bringup mm_kinect_bringup.launch camera1:=true rgbd1:=true
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 localization:=true
-- robot_description, robot_state_pusblisher and joint_state_publisher from mm_rtabmap.launch
### remote pc
roslaunch mm_slam base_rtabmap_rviz.launch rgbd1:=true

## base + arm + gripper - localization - move_base (user trial)
### robot
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSBbase
roslaunch mm_bringup mm_kinect_bringup.launch camera1:=true camera2:=true rgbd1:=true rgbd2:=true switched:=true
roslaunch mm_bringup mm_ur5_bringup.launch gui:=true
roslaunch mm_bringup mm_gripper_bringup.launch device:=/dev/ttyUSBgripper
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 localization:=true mm:=true database_path:=map_1111_4.db

### remote pc
roslaunch mm_moveit_config moveit_rviz.launch mm:=true rgbd1:=true rgbd2:=true

## base + arm + gripper - localization - move_base teb
### robot
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSBbase
roslaunch mm_bringup mm_kinect_bringup.launch camera1:=true rgbd1:=true switched:=true
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 localization:=true dwa:=false

### remote pc
roslaunch mm_moveit_config moveit_rviz.launch mm:=true rgbd1:=true

########################################################
## odometry error
** to do
### robot
roslaunch razor_imu_9dof razor-pub.launch
roslaunch mm_bringup mm_odom_pusblisher.launch
roslaunch mm_bringup description_publish.launch

### pc
roslaunch robot_pose_ekf robot_pose_ekf.launch vo_used:=false