# launch robot model
roslaunch mm_description display.launch

# keyboard command
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# move the robot
roslaunch mm_bringup mm_mobile_bringup.launch

# visulize the robot on the map
roslaunch mm_navigation mm_keyboard_navigation.launch

########################################################
########################################################
# base odom publisher
- changed v_max=0.2 and w_max=0.4
- delete test_pose
- add odom_test_pose
- add device

########################################################
# ur_moder_driver
- add mm_ur5_bringup.launch to /launch

########################################################
# razor_imu_9dof
- deleted remapping odom to /pr2~/odom
- changed subscribed topic from imu_data to imu

########################################################
# robotiq
- 

########################################################
########################################################
# keyboard teleop
## robot
roslaunch mm_bringup mm_mobile_bringup.launch
roslaunch openni_launch openni.launch camera:=kinect1

## pc
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
roslaunch mm_navigation mm_keyboard_navigation.launch

########################################################
# rgbd remote
## robot
roslaunch openni_launch openni.launch camera:=kinect1 depth_registration:=true
roslaunch mm_slam mm_combine_rgbd.launch camera:=kinect1 rviz:=false

## pc
roslaunch mm_slam mm_rgbd_remote_viz.py

########################################################
# rtabmap - slam - rviz
## robot
roslaunch openni_launch openni.launch camera:=kinect1 depth_registration:=true
roslaunch mm_bringup mm_mobile_bringup.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1

rosbag record /odom /tf /tf_static /joint_states /kinect1/depth_registered/image_raw /kinect1/depth_registered/camera_info /kinect1/rgb/image_rect_color /kinect1/rgb/camera_info /kinect_scan

# rtabmap - slam - rtabmapviz
## robot
roslaunch openni_launch openni.launch camera:=kinect1 depth_registration:=true
roslaunch mm_bringup mm_mobile_bringup.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 rviz:=false rtabmapviz:=true

########################################################
# rtabmap - slam - playback - rviz
## robot
rosbag play "<file_name>.bag"
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1
rosrun map_server map_saver map:=/rtabmap/proj_map
- tf error

########################################################
# rtabmap - slam - remote - rviz
## robot
roslaunch openni_launch openni.launch camera:=kinect1 depth_registration:=true
roslaunch mm_bringup mm_mobile_bringup.launch
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 rviz:=false

rosbag record /odom /tf /tf_static /joint_states /kinect1/depth_registered/image_raw /kinect1/depth_registered/camera_info /kinect1/rgb/image_rect_color /kinect1/rgb/camera_info /kinect_scan

## pc
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rosrun rviz rviz -d $(find mm_slam)/rviz/mm_rtabmap.rviz
rqt_console

########################################################
# keyboard teleop + visualize on the map
## robot
roslaunch openni_launch openni.launch camera:=kinect1 depth_registration:=true
roslaunch mm_bringup mm_mobile_bringup.launch
roslaunch mm_slam mm_combine_rgbd.launch camera:=kinect1 rviz:=false

## pc
roslaunch mm_navigation mm_keyboard_navigation.launch camera:=kinect1 map_file:=$HOME/test3_map.yaml

########################################################
# move_base teleop + visualize on the map
## robot
roslaunch openni_launch openni.launch camera:=kinect1 depth_registration:=true
roslaunch mm_bringup mm_mobile_bringup.launch
roslaunch mm_slam mm_rtabmap.launch camera:=kinect1 localization:=true rviz:=false rgbd:=true

## pc
rviz -d mm_move_base.rviz

########################################################
# optimize openni processing for rtabmap
1.
roslaunch openni_launch openni.launch camera:=kinect1 depth_registration:=true

2.
roslaunch openni_launch openni.launch camera:=kinect1 depth_registration:=true
rgb_processing:=true
ir_processing:=false
depth_processing:=false
depth_registered_processing:=false
disparity_proccessing:= false
disparity_registered_processing:=false
hw_registered_processing:=false
sw_registered_processing:=false

########################################################
# universal robot moveit
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.9
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch 
roslaunch ur5_moveit_config moveit_rviz.launch

########################################################
# robotiq
roslaunch robotiq_2f_85_gripper_visualization test_2f_85_model.launch
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB1
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py

########################################################
# multiple kinects
roslaunch openni_launch openni.launch device_id:=#1 camera:=kinect2 depth_registration:=true
roslaunch openni_launch openni.launch device_id:=#2 camera:=kinect1 depth_registration:=true

########################################################
# mm moveit
roslaunch ur_modern_driver mm_ur5_bringup.launch robot_ip:=192.168.1.9
roslaunch mm_moveit_config mm_moveit_planning_execution.launch 
roslaunch mm_moveit_config moveit_rviz.launch

########################################################
# imu
roslaunch razor_imu_9dof razor-pub.launch
roslaunch robot_pose_ekf.launch vo_used:=false

########################################################
# odometry error
## robot
roslaunch razor_imu_9dof razor-pub.launch
roslaunch mm_bringup mm_odom_pusblisher.launch
roslaunch mm_bringup description_publish.launch

## pc
roslaunch robot_pose_ekf robot_pose_ekf.launch vo_used:=false

########################################################
# mm moveit remote
## robot
roslaunch mm_bringup mm_mobile_bringup.launch

roslaunch openni_launch openni.launch device_id:=#1 camera:=kinect2 depth_registration:=true
roslaunch openni_launch openni.launch device_id:=#2 camera:=kinect1 depth_registration:=true

rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
rosrun robotiq_2f_gripper_control robotiq_2f_gripper_ctrl_gui.py

roslaunch ur_modern_driver mm_ur5_bringup.launch robot_ip:=192.168.1.9
roslaunch mm_moveit_config mm_moveit_planning_execution.launch 

roslaunch mm_slam mm_rtabmap.launch rgbd:=true camera:=kinect1 rviz:=false localization:=true mm_description:=false
roslaunch mm_slam mm_combine_rgbd.launch camera:=kinect2 rviz:=false ############


## pc
roslaunch mm_moveit_config moveit_rviz.launch

roslaunch mm_slam mm_rtabmap_remote_viz.launch camera:=kinect2 rviz:=false ##############

rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
rosrun teleop_twist..

########################################################
# mm moveit remote (combined)
## robot
roslaunch mm_bringup mm_mobile_bringup.launch device:=/dev/ttyUSB1
roslaunch mm_bringup mm_kinect_bringup.launch rgbd:=true
roslaunch mm_bringup mm_gripper_bringup.launch device:=/dev/ttyUSB0
roslaunch mm_bringup mm_ur5_bringup.launch
roslaunch mm_slam mm_rtabmap.launch rgbd:=false camera:=kinect1 rviz:=false localization:=true mm_description:=false
rosrun mm_moveit_config mm_moveit_gui_execution.py

## remote
roslaunch mm_moveit_config moveit_rviz.launch slam:=true
roslaunch mm_slam mm_rgbd_remote_viz.launch camera:=kinect1
roslaunch mm_slam mm_rgbd_remote_viz.launch camera:=kinect2 max_depth:=2.0

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

########################################################
ssh -X mm@192.168.0.111