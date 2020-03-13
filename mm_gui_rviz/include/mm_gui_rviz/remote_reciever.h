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

#ifndef MM_GUI_RVIZ_REMOTE_RECIEVER_H
#define MM_GUI_RVIZ_REMOTE_RECIEVER_H

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <move_base_msgs/MoveBaseActionResult.h>

#include <std_srvs/Empty.h>

#include <string>
#include <vector>

namespace mm_gui_rviz
{
class RemoteReciever
{
private:
  std::string tb_string;
  int mb_status = 0;
  int mm_step = 6;
  // int mb_result_status;
  // std::vector<float>* ik_cost(float);

public:
  RemoteReciever()
  {
    // Publisher
    mb_cancel_publisher_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    record_publisher_ = nh_.advertise<std_msgs::Bool>("pcl_record", 1);
    capture_publisher_ = nh_.advertise<std_msgs::Bool>("pcl_capture", 1);
    clear_publisher_ = nh_.advertise<std_msgs::Bool>("pcl_clear", 1);

    pick_approach_plan_publisher_ = nh_.advertise<std_msgs::Bool>("pick_approach_plan", 1);
    pick_approach_execute_publisher_ = nh_.advertise<std_msgs::Bool>("pick_approach_execute", 1);
    pick_retreat_plan_publisher_ = nh_.advertise<std_msgs::Bool>("pick_retreat_plan", 1);
    pick_retreat_execute_publisher_ = nh_.advertise<std_msgs::Bool>("pick_retreat_execute", 1);

    place_approach_plan_publisher_ = nh_.advertise<std_msgs::Bool>("place_approach_plan", 1);
    place_approach_execute_publisher_ = nh_.advertise<std_msgs::Bool>("place_approach_execute", 1);
    place_retreat_plan_publisher_ = nh_.advertise<std_msgs::Bool>("place_retreat_plan", 1);
    place_retreat_execute_publisher_ = nh_.advertise<std_msgs::Bool>("place_retreat_execute", 1);
    
    e_stop_publisher_ = nh_.advertise<std_msgs::Bool>("e_stop", 1);

    move_xp_publisher_ = nh_.advertise<std_msgs::Bool>("move_xp", 1);
    move_xm_publisher_ = nh_.advertise<std_msgs::Bool>("move_xm", 1);
    move_yp_publisher_ = nh_.advertise<std_msgs::Bool>("move_yp", 1);
    move_ym_publisher_ = nh_.advertise<std_msgs::Bool>("move_ym", 1);
    move_zp_publisher_ = nh_.advertise<std_msgs::Bool>("move_zp", 1);
    move_zm_publisher_ = nh_.advertise<std_msgs::Bool>("move_zm", 1);

    rb_publisher_ = nh_.advertise<std_msgs::String>("rotate_axis", 1);
    sl_publisher_ = nh_.advertise<std_msgs::Int32>("distance", 1);
    clear_imarker_publisher_ = nh_.advertise<std_msgs::Bool>("clear_imarker", 1);

    solution_publisher_ = nh_.advertise<std_msgs::Int32>("solution_num", 1);
    solve_ik_publisher_ = nh_.advertise<std_msgs::Bool>("solve_ik", 1);

    gripper_publisher_ = nh_.advertise<std_msgs::Bool>("gripper_close", 1);

    // Subscriber
    tb_subscriber_ = nh_.subscribe<std_msgs::String>("instruction", 5, &RemoteReciever::instruction_cb, this);
    mb_status_subscriber_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 5, &RemoteReciever::mb_status_cb, this);
    mm_step_subscriber_ = nh_.subscribe<std_msgs::Int32>("/mm_gui_step", 5, &RemoteReciever::mm_step_cb, this);

    // Service
    rtabmap_pause_client_ = nh_.serviceClient<std_srvs::Empty>("/rtabmap/pause");
    rtabmap_resume_client_ = nh_.serviceClient<std_srvs::Empty>("/rtabmap/resume");
  }

  void publishCancelMB()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "MBCancel");

    actionlib_msgs::GoalID msg;
    msg.stamp = ros::Time::now();
    mb_cancel_publisher_.publish(msg);
  }

  void publishEStop(bool data)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "EStop");

    std_msgs::Bool msg;
    msg.data = data;
    e_stop_publisher_.publish(msg);
  }

  void publishGripperOpen()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "GripperOpen");

    std_msgs::Bool msg;
    msg.data = false;
    gripper_publisher_.publish(msg);
  }

  void publishGripperClose()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "GripperClose");

    std_msgs::Bool msg;
    msg.data = true;
    gripper_publisher_.publish(msg);
  }

  void publishCmdVel(geometry_msgs::Twist msg)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "CmdVel");
    cmd_vel_publisher_.publish(msg);
  }

  void publishPclRecord()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PclRecord");
    
    std_msgs::Bool msg;
    msg.data = true;
    record_publisher_.publish(msg);
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

  void publishPickApproachPlan()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PickApproachPlan");

    std_msgs::Bool msg;
    msg.data = true;
    pick_approach_plan_publisher_.publish(msg);
  }

  void publishPickApproachExecute()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PickApproachExecute");

    std_msgs::Bool msg;
    msg.data = true;
    pick_approach_execute_publisher_.publish(msg);
  }

  void publishPickRetreatPlan()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PickRetreatPlan");

    std_msgs::Bool msg;
    msg.data = true;
    pick_retreat_plan_publisher_.publish(msg);
  }

  void publishPickRetreatExecute()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PickRetreatExecute");

    std_msgs::Bool msg;
    msg.data = true;
    pick_retreat_execute_publisher_.publish(msg);
  }

  void publishPlaceApproachPlan()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PlaceApproachPlan");

    std_msgs::Bool msg;
    msg.data = true;
    place_approach_plan_publisher_.publish(msg);
  }

  void publishPlaceApproachExecute()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PlaceApproachExecute");

    std_msgs::Bool msg;
    msg.data = true;
    place_approach_execute_publisher_.publish(msg);
  }

  void publishPlaceRetreatPlan()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PlaceRetreatPlan");

    std_msgs::Bool msg;
    msg.data = true;
    place_retreat_plan_publisher_.publish(msg);
  }

  void publishPlaceRetreatExecute()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "PlaceRetreatExecute");

    std_msgs::Bool msg;
    msg.data = true;
    place_retreat_execute_publisher_.publish(msg);
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

  void publishRB(int value)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "RB");

    std_msgs::String msg;
    
    if (value == 1)      msg.data = "x";
    else if (value == 2) msg.data = "y";
    else if (value == 3) msg.data = "z";

    rb_publisher_.publish(msg);  
  }

  void publishDistance(int value)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Distance");

    std_msgs::Int32 msg;
    msg.data = value;
    sl_publisher_.publish(msg);
  }

  void publishClearIMarker()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "ClearIMarker");

    std_msgs::Bool msg;
    msg.data = true;
    clear_imarker_publisher_.publish(msg);  
  }

  void publishSolution(int value)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "SolutionNum");

    std_msgs::Int32 msg;
    msg.data = value;
    solution_publisher_.publish(msg);    
  }

  void publishSolveIK(bool value)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "SolveIK");

    std_msgs::Bool msg;
    msg.data = value;
    solve_ik_publisher_.publish(msg);
  }

  void callPauseRtabmap()
  {
    std_srvs::Empty srv;
    if(rtabmap_pause_client_.call(srv))
      ROS_DEBUG_STREAM_NAMED("gui", "rtabmap pause success");
    else
      ROS_DEBUG_STREAM_NAMED("gui", "rtabmap pause failed");
  }

  void callResumeRtabmap()
  {
    std_srvs::Empty srv;
    if(rtabmap_resume_client_.call(srv))
      ROS_DEBUG_STREAM_NAMED("gui", "rtabmap resume success");
    else
      ROS_DEBUG_STREAM_NAMED("gui", "rtabmap resume failed");
  }

  void instruction_cb(const std_msgs::String::ConstPtr& msg)
  {
    tb_string = msg->data.c_str();
  }

  // void mb_result_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
  // {
  //   mb_result_status = msg->status.status;
  // }

  void mb_status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
  {
    if (!msg->status_list.empty())
      mb_status = msg->status_list[0].status;
  }

  void mm_step_cb(const std_msgs::Int32::ConstPtr& msg)
  {
    mm_step = msg->data;
  }
  

  const std::string& get_instruction() {return tb_string;}

  // const int& get_mb_result() {return mb_result_status;}
  const int& get_mb_status() {return mb_status;}

  const int& get_mm_step() {return mm_step;}

protected:
  // The ROS publishers
  ros::Publisher mb_cancel_publisher_;

  ros::Publisher base_stop_publisher_;
  ros::Publisher gripper_publisher_;
  ros::Publisher cmd_vel_publisher_;
  
  ros::Publisher record_publisher_;
  ros::Publisher capture_publisher_;
  ros::Publisher clear_publisher_;
  
  ros::Publisher pick_approach_plan_publisher_;
  ros::Publisher pick_approach_execute_publisher_;
  ros::Publisher pick_retreat_plan_publisher_;
  ros::Publisher pick_retreat_execute_publisher_;
  ros::Publisher place_approach_plan_publisher_;
  ros::Publisher place_approach_execute_publisher_;
  ros::Publisher place_retreat_plan_publisher_;
  ros::Publisher place_retreat_execute_publisher_;

  ros::Publisher e_stop_publisher_;

  ros::Publisher move_xp_publisher_;
  ros::Publisher move_yp_publisher_;
  ros::Publisher move_zp_publisher_;
  ros::Publisher move_xm_publisher_;
  ros::Publisher move_ym_publisher_;
  ros::Publisher move_zm_publisher_;

  ros::Publisher rb_publisher_;
  ros::Publisher sl_publisher_;
  ros::Publisher clear_imarker_publisher_;

  ros::Publisher solution_publisher_;
  ros::Publisher solve_ik_publisher_;

  ros::Subscriber tb_subscriber_;
  ros::Subscriber mb_status_subscriber_;
  ros::Subscriber mm_step_subscriber_;

  ros::ServiceClient rtabmap_pause_client_;
  ros::ServiceClient rtabmap_resume_client_;
  
  // The ROS node handle.
  ros::NodeHandle nh_;

};

}  // end namespace mm_gui_rviz

#endif  // MM_GUI_RVIZ_REMOTE_RECIEVER_H
