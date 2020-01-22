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

#include <cstdio>
#include <string>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QFrame>

#include "mm_gui_rviz_demo.h"

namespace mm_gui_rviz_demo
{
MMGuiRvizDemo::MMGuiRvizDemo(QWidget* parent) : rviz::Panel(parent)
{
  //////////////////////////////
  // Button
  //////////////////////////////
  
  // Create a push button
  btn_emergency_stop_ = new QPushButton(this);
  btn_emergency_stop_->setText("Stop");
  connect(btn_emergency_stop_, SIGNAL(clicked()), this, SLOT(emergencyStop()));

  // Create a push button
  btn_gripper_open_ = new QPushButton(this);
  btn_gripper_open_->setText("Open");
  connect(btn_gripper_open_, SIGNAL(clicked()), this, SLOT(moveGripperOpen()));

  // Create a push button
  btn_gripper_close_ = new QPushButton(this);
  btn_gripper_close_->setText("Close");
  connect(btn_gripper_close_, SIGNAL(clicked()), this, SLOT(moveGripperClose()));

  // Create a push button
  btn_pcl_capture_ = new QPushButton(this);
  btn_pcl_capture_->setText("Capture");
  connect(btn_pcl_capture_, SIGNAL(clicked()), this, SLOT(pclCapture()));

  // Create a push button
  btn_pcl_clear_ = new QPushButton(this);
  btn_pcl_clear_->setText("Clear");
  connect(btn_pcl_clear_, SIGNAL(clicked()), this, SLOT(pclClear()));

  // Create a push button
  btn_approach_plan_ = new QPushButton(this);
  btn_approach_plan_->setText("Plan");
  connect(btn_approach_plan_, SIGNAL(clicked()), this, SLOT(approachArmPlan()));

  // Create a push button
  btn_approach_execute_ = new QPushButton(this);
  btn_approach_execute_->setText("Execute");
  connect(btn_approach_execute_, SIGNAL(clicked()), this, SLOT(approachArmExcute()));

  // Create a push button
  btn_move_xp_ = new QPushButton(this);
  btn_move_xp_->setText("+");
  connect(btn_move_xp_, SIGNAL(clicked()), this, SLOT(moveXP()));

  // Create a push button
  btn_move_xm_ = new QPushButton(this);
  btn_move_xm_->setText("-");
  connect(btn_move_xm_, SIGNAL(clicked()), this, SLOT(moveXM()));

  // Create a push button
  btn_move_yp_ = new QPushButton(this);
  btn_move_yp_->setText("+");
  connect(btn_move_yp_, SIGNAL(clicked()), this, SLOT(moveYP()));

  // Create a push button
  btn_move_ym_ = new QPushButton(this);
  btn_move_ym_->setText("-");
  connect(btn_move_ym_, SIGNAL(clicked()), this, SLOT(moveYM()));

  // Create a push button
  btn_move_zp_ = new QPushButton(this);
  btn_move_zp_->setText("+");
  connect(btn_move_zp_, SIGNAL(clicked()), this, SLOT(moveZP()));

  // Create a push button
  btn_move_zm_ = new QPushButton(this);
  btn_move_zm_->setText("-");
  connect(btn_move_zm_, SIGNAL(clicked()), this, SLOT(moveZM()));

  // Create a push button
  btn_waipoint_add_ = new QPushButton("add", this);
  btn_waipoint_remove_ = new QPushButton("remove", this);
  btn_interpolation_compute_ = new QPushButton("interpolate", this);
  btn_interpolation_execute_ = new QPushButton("execute", this);
  
  connect(btn_waipoint_add_, SIGNAL(clicked()), this, SLOT(addWaypoint()));  
  connect(btn_waipoint_remove_, SIGNAL(clicked()), this, SLOT(removeWaypoint()));
  connect(btn_interpolation_compute_, SIGNAL(clicked()), this, SLOT(computeInterpolation()));
  connect(btn_interpolation_execute_, SIGNAL(clicked()), this, SLOT(executeInterpolation()));


  //////////////////////////////
  // Layout
  //////////////////////////////

  // Main Layout
  layout = new QVBoxLayout;

  addTitle("Capture Point Cloud Data");
  layout->addWidget(btn_pcl_capture_);
  layout->addWidget(btn_pcl_clear_);

  addTitle("Approach to Clicked Point");
  layout->addWidget(btn_approach_plan_);
  layout->addWidget(btn_approach_execute_);

  // Sub Layout
  sub_layout = new QHBoxLayout;
  subx_layout = new QVBoxLayout;
  suby_layout = new QVBoxLayout;
  subz_layout = new QVBoxLayout;

  addTitle("UR5 Actions");  
  subx_layout->addWidget(btn_move_xp_);
  addLabelX();
  subx_layout->addWidget(btn_move_xm_);

  suby_layout->addWidget(btn_move_yp_);
  addLabelY();
  suby_layout->addWidget(btn_move_ym_);

  subz_layout->addWidget(btn_move_zp_);
  addLabelZ();
  subz_layout->addWidget(btn_move_zm_);

  sub_layout->addLayout(subx_layout);
  sub_layout->addLayout(suby_layout);
  sub_layout->addLayout(subz_layout);
  layout->addLayout(sub_layout);
  
  addTitle("Add or Remove Movable Waypoints");
  layout->addWidget(btn_waipoint_add_);
  layout->addWidget(btn_waipoint_remove_);

  addTitle("Traverse Waypoints");
  layout->addWidget(btn_interpolation_compute_);
  layout->addWidget(btn_interpolation_execute_);

  addTitle("Gripper Actions");
  layout->addWidget(btn_gripper_open_);
  layout->addWidget(btn_gripper_close_);

  addTitle("Emergency");
  layout->addWidget(btn_emergency_stop_);

  setLayout(layout);

  btn_pcl_capture_->setEnabled(true);
  btn_pcl_clear_->setEnabled(true);
  btn_approach_plan_->setEnabled(true);
  btn_approach_execute_->setEnabled(true);
  btn_move_xp_->setEnabled(true);
  btn_move_xm_->setEnabled(true);
  btn_move_yp_->setEnabled(true);
  btn_move_ym_->setEnabled(true);
  btn_move_zp_->setEnabled(true);
  btn_move_zm_->setEnabled(true);

  btn_waipoint_add_->setEnabled(true);
  btn_waipoint_remove_->setEnabled(true);
  btn_interpolation_compute_->setEnabled(true);
  btn_interpolation_execute_->setEnabled(true);

  btn_gripper_open_->setEnabled(true);
  btn_gripper_close_->setEnabled(true);
  btn_emergency_stop_->setEnabled(true);
}

void MMGuiRvizDemo::addLabelX()
{
  QLabel* label = new QLabel(QString::fromStdString("x"));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  subx_layout->addWidget(label);
}

void MMGuiRvizDemo::addLabelY()
{
  QLabel* label = new QLabel(QString::fromStdString("y"));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  suby_layout->addWidget(label);
}

void MMGuiRvizDemo::addLabelZ()
{
  QLabel* label = new QLabel(QString::fromStdString("z"));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  subz_layout->addWidget(label);
}

void MMGuiRvizDemo::addTitle(std::string str)
{
  QLabel* label = new QLabel(QString::fromStdString(str));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  layout->addWidget(label);
}

void MMGuiRvizDemo::emergencyStop()
{
  remote_reciever_.publishEmergencyStop();
}

void MMGuiRvizDemo::moveGripperOpen()
{
  remote_reciever_.publishGripperOpen();
}

void MMGuiRvizDemo::moveGripperClose()
{
  remote_reciever_.publishGripperClose();
}

void MMGuiRvizDemo::pclCapture()
{
  remote_reciever_.publishPclCapture();
}

void MMGuiRvizDemo::pclClear()
{
  remote_reciever_.publishPclClear();
}

void MMGuiRvizDemo::approachArmPlan()
{
  remote_reciever_.publishApproachPlan();
}

void MMGuiRvizDemo::approachArmExcute()
{
  remote_reciever_.publishApproachExcute();
}

void MMGuiRvizDemo::moveXP()
{
  remote_reciever_.publishMoveXP();
}

void MMGuiRvizDemo::moveXM()
{
  remote_reciever_.publishMoveXM();
}

void MMGuiRvizDemo::moveYP()
{
  remote_reciever_.publishMoveYP();
}

void MMGuiRvizDemo::moveYM()
{
  remote_reciever_.publishMoveYM();
}

void MMGuiRvizDemo::moveZP()
{
  remote_reciever_.publishMoveZP();
}

void MMGuiRvizDemo::moveZM()
{
  remote_reciever_.publishMoveZM();
}

void MMGuiRvizDemo::addWaypoint()
{
  remote_reciever_.publishAddWaypoint();
}

void MMGuiRvizDemo::removeWaypoint()
{
  remote_reciever_.publishRemoveWaypoint();
}

void MMGuiRvizDemo::computeInterpolation()
{
  remote_reciever_.publishComputeInterpolation();
}

void MMGuiRvizDemo::executeInterpolation()
{
  remote_reciever_.publishExecuteInterpolation();
}


void MMGuiRvizDemo::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void MMGuiRvizDemo::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace mm_gui_rviz_demo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mm_gui_rviz_demo::MMGuiRvizDemo, rviz::Panel)
