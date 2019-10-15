# basic commands
## ssh
ssh -X mm@192.168.0.111

## visualize the robot model
roslaunch mm_description display.launch

## keyboard command
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

## rgbd remote
roslaunch mm_slam mm_combine_rgbd.launch camera:=kinect1 rviz:=false
roslaunch mm_slam mm_rgbd_remote_viz.launch camera:=kinect1

## bringup the robot (default)
### base
rosrun mm_bringup base_odom_publisher.py
### multiple kinect
roslaunch openni_launch openni.launch device_id:=#1 camera:=kinect1 depth_registration:=true
roslaunch openni_launch openni.launch device_id:=#2 camera:=kinect2 depth_registration:=true
### arm (MoveIt!)
roslaunch ur_modern_driver mm_ur5_bringup.launch robot_ip:=192.168.1.9
roslaunch mm_moveit_config mm_moveit_planning_execution.launch 
roslaunch mm_moveit_config moveit_rviz.launch
### gripper
roslaunch robotiq_2f_85_gripper_visualization test_2f_85_model.launch
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB1
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py
### imu
roslaunch razor_imu_9dof razor-pub.launch
roslaunch robot_pose_ekf.launch vo_used:=false

## move the robot (compressed)
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSB0
roslaunch mm_bringup mm_kinect_bringup.launch rgbd:=true
roslaunch mm_bringup mm_ur5_bringup.launch
roslaunch mm_bringup mm_gripper_bringup.launch device:=/dev/ttyUSB1


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
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSB0
roslaunch openni_launch openni.launch device_id:=#1 camera:=kinect1 depth_registration:=true
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 mm:=false

### remote pc
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rosrun rviz rviz -d $(find mm_slam)/rviz/mm_rtabmap.rviz
rosbag record /odom /tf /tf_static /joint_states /kinect1/depth_registered/image_raw /kinect1/depth_registered/camera_info /kinect1/rgb/image_rect_color /kinect1/rgb/camera_info /kinect_scan
rqt_console

## base - slam playback
rosbag play "<file_name>.bag"
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 mm:=false rviz:=true 
rosrun map_server map_saver map:=/rtabmap/proj_map
** tf error

## base - localization - move_base
### robot
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSB0
roslaunch openni_launch openni.launch device_id:=#1 camera:=kinect1 depth_registration:=true
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 localization:=true mm:=false

### remote pc
rviz -d mm_move_base.rviz

## base + arm + gripper - localization - move_base
## robot
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSB0
roslaunch mm_bringup mm_kinect_bringup.launch rgbd:=true
roslaunch mm_bringup mm_ur5_bringup.launch
roslaunch mm_bringup mm_gripper_bringup.launch device:=/dev/ttyUSB1
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 localization:=true
rosrun mm_moveit_config mm_moveit_gui_execution.py

## remote pc
roslaunch mm_moveit_config moveit_rviz.launch slam:=true
roslaunch mm_slam mm_rgbd_remote_viz.launch camera:=kinect1
roslaunch mm_slam mm_rgbd_remote_viz.launch camera:=kinect2 max_depth:=2.0

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

########################################################
## odometry error
** to do
### robot
roslaunch razor_imu_9dof razor-pub.launch
roslaunch mm_bringup mm_odom_pusblisher.launch
roslaunch mm_bringup description_publish.launch

### pc
roslaunch robot_pose_ekf robot_pose_ekf.launch vo_used:=false