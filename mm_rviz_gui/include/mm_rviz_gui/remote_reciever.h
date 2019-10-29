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

#ifndef MM_RVIZ_GUI_REMOTE_RECIEVER_H
#define MM_RVIZ_GUI_REMOTE_RECIEVER_H

// #include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace mm_rviz_gui
{
class RemoteReciever
{
public:
  RemoteReciever()
  {
    // joy_publisher_ = nh_.advertise<sensor_msgs::Joy>("/rviz_gui_joy", 1);
    base_stop_publisher_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    plan_publisher_ = nh_.advertise<std_msgs::Bool>("ur5_plan", 1);
    execute_publisher_ = nh_.advertise<std_msgs::Bool>("ur5_execute", 1);
    stop_publisher_ = nh_.advertise<std_msgs::Bool>("ur5_stop", 1);
    gripper_publisher_ = nh_.advertise<std_msgs::Bool>("gripper_joint", 1);
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  void publishBaseStop()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "BaseStop");
    // sensor_msgs::Joy msg;
    // msg.buttons.resize(9);
    // msg.buttons[1] = 1;
    // joy_publisher_.publish(msg);

    actionlib_msgs::GoalID msg;
    msg.id = "";
    base_stop_publisher_.publish(msg);
  }

  void publishArmPlan()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "ArmPlan");
    // sensor_msgs::Joy msg;
    // msg.buttons.resize(9);
    // msg.buttons[2] = 1;
    // joy_publisher_.publish(msg);
    
    std_msgs::Bool msg;
    msg.data = true;
    plan_publisher_.publish(msg);
  }

  void publishArmExecute()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "ArmExecute");

    // sensor_msgs::Joy msg;
    // msg.buttons.resize(9);
    // msg.buttons[3] = 1;
    // joy_publisher_.publish(msg);

    std_msgs::Bool msg;
    msg.data = true;
    execute_publisher_.publish(msg);
  }

  void publishArmStop()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "ArmStop");

    // sensor_msgs::Joy msg;
    // msg.buttons.resize(9);
    // msg.buttons[4] = 1;
    // joy_publisher_.publish(msg);

    std_msgs::Bool msg;
    msg.data = true;
    stop_publisher_.publish(msg);
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

protected:
  // The ROS publishers
  // ros::Publisher joy_publisher_;
  ros::Publisher base_stop_publisher_;
  ros::Publisher plan_publisher_;
  ros::Publisher execute_publisher_;
  ros::Publisher stop_publisher_;
  ros::Publisher gripper_publisher_;
  ros::Publisher cmd_vel_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

}  // end namespace mm_rviz_gui

#endif  // MM_RVIZ_GUI_REMOTE_RECIEVER_H
