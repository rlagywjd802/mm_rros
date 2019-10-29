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

#include "mm_rviz_gui.h"

namespace mm_rviz_gui
{
MMRvizGui::MMRvizGui(QWidget* parent) : rviz::Panel(parent)
{
  // Create a push button
  btn_base_stop_ = new QPushButton(this);
  btn_base_stop_->setText("Stop");
  connect(btn_base_stop_, SIGNAL(clicked()), this, SLOT(moveBaseStop()));

  // Create a push button
  btn_arm_plan_ = new QPushButton(this);
  btn_arm_plan_->setText("Plan");
  connect(btn_arm_plan_, SIGNAL(clicked()), this, SLOT(moveArmPlan()));

  // Create a push button
  btn_arm_execute_ = new QPushButton(this);
  btn_arm_execute_->setText("Execute");
  connect(btn_arm_execute_, SIGNAL(clicked()), this, SLOT(moveArmExecute()));

  // Create a push button
  btn_arm_stop_ = new QPushButton(this);
  btn_arm_stop_->setText("Stop");
  connect(btn_arm_stop_, SIGNAL(clicked()), this, SLOT(moveArmStop()));

  // Create a push button
  btn_gripper_open_ = new QPushButton(this);
  btn_gripper_open_->setText("Open");
  connect(btn_gripper_open_, SIGNAL(clicked()), this, SLOT(moveGripperOpen()));

  // Create a push button
  btn_gripper_close_ = new QPushButton(this);
  btn_gripper_close_->setText("Close");
  connect(btn_gripper_close_, SIGNAL(clicked()), this, SLOT(moveGripperClose()));

  // Layout
  layout = new QVBoxLayout;
  addTitle("Base Actions");
  layout->addWidget(btn_base_stop_);
  addTitle("UR5 Actions");
  layout->addWidget(btn_arm_plan_);
  layout->addWidget(btn_arm_execute_);
  layout->addWidget(btn_arm_stop_);
  addTitle("Gripper Actions");
  layout->addWidget(btn_gripper_open_);
  layout->addWidget(btn_gripper_close_);

  setLayout(layout);

  btn_base_stop_->setEnabled(true);
  btn_arm_plan_->setEnabled(true);
  btn_arm_execute_->setEnabled(true);
  btn_arm_stop_->setEnabled(true);
  btn_gripper_open_->setEnabled(true);
  btn_gripper_close_->setEnabled(true);
}

void MMRvizGui::addTitle(std::string str)
{
  QLabel* label = new QLabel(QString::fromStdString(str));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  layout->addWidget(label);
}

void MMRvizGui::moveBaseStop()
{
  remote_reciever_.publishBaseStop();
}

void MMRvizGui::moveArmPlan()
{
  remote_reciever_.publishArmPlan();
}

void MMRvizGui::moveArmExecute()
{
  remote_reciever_.publishArmExecute();
}

void MMRvizGui::moveArmStop()
{
  remote_reciever_.publishArmStop();
}

void MMRvizGui::moveGripperOpen()
{
  remote_reciever_.publishGripperOpen();
}

void MMRvizGui::moveGripperClose()
{
  remote_reciever_.publishGripperClose();
}

void MMRvizGui::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void MMRvizGui::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace mm_rviz_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mm_rviz_gui::MMRvizGui, rviz::Panel)
