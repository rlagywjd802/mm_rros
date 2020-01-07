/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef MM_GUI_RVIZ_DEMO_REMOTE_RECIEVER_H
#define MM_GUI_RVIZ_DEMO_REMOTE_RECIEVER_H

#include <std_msgs/Bool.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace mm_gui_rviz_demo
{
class RemoteReciever
{
public:
  RemoteReciever()
  {
    // joy_publisher_ = nh_.advertise<sensor_msgs::Joy>("/rviz_gui_joy", 1);
    base_stop_publisher_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    gripper_publisher_ = nh_.advertise<std_msgs::Bool>("gripper_joint", 1);
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    capture_publisher_ = nh_.advertise<std_msgs::Bool>("pcl_capture", 1);
    clear_publisher_ = nh_.advertise<std_msgs::Bool>("pcl_clear", 1);

    approach_plan_publisher_ = nh_.advertise<std_msgs::Bool>("approach_plan", 1);
    approach_execute_publisher_ = nh_.advertise<std_msgs::Bool>("approach_execute", 1);
    approach_stop_publisher_ = nh_.advertise<std_msgs::Bool>("approach_stop", 1);

    move_xp_publisher_ = nh_.advertise<std_msgs::Bool>("move_xp", 1);
    move_xm_publisher_ = nh_.advertise<std_msgs::Bool>("move_xm", 1);
    move_yp_publisher_ = nh_.advertise<std_msgs::Bool>("move_yp", 1);
    move_ym_publisher_ = nh_.advertise<std_msgs::Bool>("move_ym", 1);
    move_zp_publisher_ = nh_.advertise<std_msgs::Bool>("move_zp", 1);
    move_zm_publisher_ = nh_.advertise<std_msgs::Bool>("move_zm", 1);

  }

  void publishEmergencyStop()
  {
    // ROS_DEBUG_STREAM_NAMED("gui", "BaseStop");
    // // sensor_msgs::Joy msg;
    // // msg.buttons.resize(9);
    // // msg.buttons[1] = 1;
    // // joy_publisher_.publish(msg);

    // actionlib_msgs::GoalID msg;
    // msg.id = "";
    // base_stop_publisher_.publish(msg);

    ROS_DEBUG_STREAM_NAMED("gui", "ApproachStop");

    std_msgs::Bool msg;
    msg.data = true;
    approach_stop_publisher_.publish(msg);
  }

  void publishGripperOpen()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "GripperOpen");

    // sensor_msgs::Joy msg;
    // msg.buttons.resize(9);
    // msg.buttons[5] = 1;
    // joy_publisher_.publish(msg);

    std_msgs::Bool msg;
    msg.data = false;
    gripper_publisher_.publish(msg);
  }

  void publishGripperClose()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "GripperClose");

    // sensor_msgs::Joy msg;
    // msg.buttons.resize(9);
    // msg.buttons[6] = 1;
    // joy_publisher_.publish(msg);

    std_msgs::Bool msg;
    msg.data = true;
    gripper_publisher_.publish(msg);
  }

  void publishCmdVel(geometry_msgs::Twist msg)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "CmdVel");
    cmd_vel_publisher_.publish(msg);
  }

  void publishPclCapture()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PclCapture");
    
    std_msgs::Bool msg;
    msg.data = true;
    capture_publisher_.publish(msg);
  }

  void publishPclClear()
  {
  	ROS_DEBUG_STREAM_NAMED("gui", "PclClear");
    
    std_msgs::Bool msg;
    msg.data = true;
    clear_publisher_.publish(msg);
  }

  void publishApproachPlan()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "ApproachPlan");

    std_msgs::Bool msg;
    msg.data = true;
    approach_plan_publisher_.publish(msg);
  }

  void publishApproachExcute()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "ApproachExecute");

    std_msgs::Bool msg;
    msg.data = true;
    approach_execute_publisher_.publish(msg);
  }

  void publishApproachStop()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "ApproachStop");

    std_msgs::Bool msg;
    msg.data = true;
    approach_stop_publisher_.publish(msg);
  }

  void publishMoveXP()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "MoveXP");

    std_msgs::Bool msg;
    msg.data = true;
    move_xp_publisher_.publish(msg);
  }

  void publishMoveXM()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "MoveXM");

    std_msgs::Bool msg;
    msg.data = true;
    move_xm_publisher_.publish(msg);
  }

  void publishMoveYP()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "MoveYP");

    std_msgs::Bool msg;
    msg.data = true;
    move_yp_publisher_.publish(msg);
  }

  void publishMoveYM()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "MoveYM");

    std_msgs::Bool msg;
    msg.data = true;
    move_ym_publisher_.publish(msg);
  }

  void publishMoveZP()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "MoveZP");

    std_msgs::Bool msg;
    msg.data = true;
    move_zp_publisher_.publish(msg);
  }

  void publishMoveZM()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "MoveZM");

    std_msgs::Bool msg;
    msg.data = true;
    move_zm_publisher_.publish(msg);
  }

protected:
  // The ROS publishers
  // ros::Publisher joy_publisher_;
  ros::Publisher base_stop_publisher_;
  ros::Publisher gripper_publisher_;
  ros::Publisher cmd_vel_publisher_;
  ros::Publisher capture_publisher_;
  ros::Publisher clear_publisher_;
  
  ros::Publisher approach_plan_publisher_;
  ros::Publisher approach_execute_publisher_;
  ros::Publisher approach_stop_publisher_;

  ros::Publisher move_xp_publisher_;
  ros::Publisher move_yp_publisher_;
  ros::Publisher move_zp_publisher_;
  ros::Publisher move_xm_publisher_;
  ros::Publisher move_ym_publisher_;
  ros::Publisher move_zm_publisher_;


  // The ROS node handle.
  ros::NodeHandle nh_;
};

}  // end namespace mm_gui_rviz_demo

#endif  // MM_GUI_RVIZ_DEMO_REMOTE_RECIEVER_H