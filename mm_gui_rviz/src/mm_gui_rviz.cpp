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
#include <vector>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QFrame>
#include <QMessageBox>

#include "mm_gui_rviz.h"

namespace mm_gui_rviz
{
MMGuiRviz::MMGuiRviz(QWidget* parent) : rviz::Panel(parent)
{
  //////////////////////////////
  // Button
  //////////////////////////////
  
  // Create a push button
  btn_mb_cancel_ = new QPushButton(this);
  btn_mb_cancel_->setText("Cancel motion planner");
  connect(btn_mb_cancel_, SIGNAL(clicked()), this, SLOT(cancelMB()));

  // Create a push button
  btn_rtabmap_pause_ = new QPushButton(this);
  btn_rtabmap_pause_->setText("Pause");
  connect(btn_rtabmap_pause_, SIGNAL(clicked()), this, SLOT(pauseRtabmap()));

  // Create a push button
  btn_rtabmap_resume_ = new QPushButton(this);
  btn_rtabmap_resume_->setText("Resume");
  connect(btn_rtabmap_resume_, SIGNAL(clicked()), this, SLOT(resumeRtabmap()));

  // Create a push button
  btn_pcl_record_ = new QPushButton(this);
  btn_pcl_record_->setText("Capture");
  connect(btn_pcl_record_, SIGNAL(clicked()), this, SLOT(pclRecord()));

  // Create a push button
  btn_pcl_capture_ = new QPushButton(this);
  btn_pcl_capture_->setText("Show Point Cloud");
  connect(btn_pcl_capture_, SIGNAL(clicked()), this, SLOT(pclCapture()));

  // Create a push button
  btn_pcl_clear_ = new QPushButton(this);
  btn_pcl_clear_->setText("Clear");
  connect(btn_pcl_clear_, SIGNAL(clicked()), this, SLOT(pclClear()));

  // Create a push button
  btn_gripper_open_ = new QPushButton(this);
  btn_gripper_open_->setText("Open");
  connect(btn_gripper_open_, SIGNAL(clicked()), this, SLOT(moveGripperOpen()));

  // Create a push button
  btn_gripper_close_ = new QPushButton(this);
  btn_gripper_close_->setText("Close");
  connect(btn_gripper_close_, SIGNAL(clicked()), this, SLOT(moveGripperClose()));

  // Create a push button
  btn_appr_plan_ = new QPushButton(this);
  btn_appr_plan_->setText("Plan");
  connect(btn_appr_plan_, SIGNAL(clicked()), this, SLOT(apprPlan()));

  // Create a push button
  btn_appr_execute_ = new QPushButton(this);
  btn_appr_execute_->setText("Execute");
  connect(btn_appr_execute_, SIGNAL(clicked()), this, SLOT(apprExecute()));

  // Create a push button
  btn_retr_plan_ = new QPushButton(this);
  btn_retr_plan_->setText("Plan");
  connect(btn_retr_plan_, SIGNAL(clicked()), this, SLOT(retrPlan()));

  // Create a push button
  btn_retr_execute_ = new QPushButton(this);
  btn_retr_execute_->setText("Execute");
  connect(btn_retr_execute_, SIGNAL(clicked()), this, SLOT(retrExecute()));

  // Create a push button
  btn_move_xp_ = new QPushButton(this);
  btn_move_xp_->setText("Forward");
  connect(btn_move_xp_, SIGNAL(clicked()), this, SLOT(moveXP()));

  // Create a push button
  btn_move_xm_ = new QPushButton(this);
  btn_move_xm_->setText("Backward");
  connect(btn_move_xm_, SIGNAL(clicked()), this, SLOT(moveXM()));

  // Create a push button
  btn_move_yp_ = new QPushButton(this);
  btn_move_yp_->setText("Left");
  connect(btn_move_yp_, SIGNAL(clicked()), this, SLOT(moveYP()));

  // Create a push button
  btn_move_ym_ = new QPushButton(this);
  btn_move_ym_->setText("Right");
  connect(btn_move_ym_, SIGNAL(clicked()), this, SLOT(moveYM()));

  // Create a push button
  btn_move_zp_ = new QPushButton(this);
  btn_move_zp_->setText("Up");
  connect(btn_move_zp_, SIGNAL(clicked()), this, SLOT(moveZP()));

  // Create a push button
  btn_move_zm_ = new QPushButton(this);
  btn_move_zm_->setText("Down");
  connect(btn_move_zm_, SIGNAL(clicked()), this, SLOT(moveZM()));

  // Create a radio button
  rbtn_appr_1_ = new QRadioButton("roll(red)", this);
  rbtn_appr_2_ = new QRadioButton("pitch(green)", this);
  rbtn_appr_3_ = new QRadioButton("yaw(blue)", this);
  connect(rbtn_appr_1_, SIGNAL(clicked()), this, SLOT(apprRB1()));  
  connect(rbtn_appr_2_, SIGNAL(clicked()), this, SLOT(apprRB2()));  
  connect(rbtn_appr_3_, SIGNAL(clicked()), this, SLOT(apprRB3()));
  
  // Create a slider
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setMinimum(0);
  slider_->setMaximum(20);
  slider_->setValue(10);
  connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(setDistance(int)));

  // Create a push button
  btn_imarker_clear_ = new QPushButton(this);
  btn_imarker_clear_->setText("Clear");
  connect(btn_imarker_clear_, SIGNAL(clicked()), this, SLOT(clearIMarker()));

  // Create a radio button
  rbtn_motion_1_ = new QRadioButton("pick up", this);
  rbtn_motion_2_ = new QRadioButton("place", this);
  connect(rbtn_motion_1_, SIGNAL(clicked()), this, SLOT(motionRB1()));  
  connect(rbtn_motion_2_, SIGNAL(clicked()), this, SLOT(motionRB2()));

  // Create a text browser
  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateText()));
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateIKCost()));
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateMBResult()));
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateMMStep()));
  timer_->start(100);

  text_browser_ = new QTextBrowser(this);
  text_browser_->setStyleSheet("font: 15pt");

  // Create a combo box
  combo_box_ = new QComboBox(this);
  combo_box_->addItem("-----");
  combo_box_->addItem("sol 0");
  combo_box_->addItem("sol 1");
  combo_box_->addItem("sol 2");
  combo_box_->addItem("sol 3");
  combo_box_->addItem("sol 4");
  combo_box_->addItem("sol 5");
  combo_box_->addItem("sol 6");
  combo_box_->addItem("sol 7");
  connect(combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(selectSolution(int)));


  //////////////////////////////
  // Layout
  //////////////////////////////

  // Main Layout
  layout = new QVBoxLayout;

  // text browser
  layout->addWidget(text_browser_);

  // Mobile Base
  lb_mb_ = new QLabel(QString::fromStdString("Mobile Base"));
  lb_mb_->setAlignment(Qt::AlignCenter);
  layout->addWidget(lb_mb_);
  layout->addWidget(btn_mb_cancel_);

  QHBoxLayout* l_rtabmap = new QHBoxLayout;
  lb_mb_loc_ = new QLabel(QString::fromStdString("localization:"));
  l_rtabmap->addWidget(lb_mb_loc_);
  l_rtabmap->addWidget(btn_rtabmap_pause_);
  l_rtabmap->addWidget(btn_rtabmap_resume_);
  layout->addLayout(l_rtabmap);

  // Camera on Manipulator
  lb_pcl_ = new QLabel(QString::fromStdString("Camera on Manipulator"));
  lb_pcl_->setAlignment(Qt::AlignCenter);
  QHBoxLayout* l_pcl = new QHBoxLayout;
  l_pcl->addWidget(btn_pcl_record_);
  l_pcl->addWidget(btn_pcl_capture_);
  l_pcl->addWidget(btn_pcl_clear_);
  layout->addWidget(lb_pcl_);
  layout->addLayout(l_pcl);

  // Settings for Approaching Pickup
  lb_appr_ = new QLabel(QString::fromStdString("Settings for Approaching Pickup"));
  lb_appr_->setAlignment(Qt::AlignCenter);
  layout->addWidget(lb_appr_);

  QHBoxLayout* l_appr_pose = new QHBoxLayout;
  lb_appr_pose_ = new QLabel(QString::fromStdString("set pre-grasp pose"));
  l_appr_pose->addWidget(rbtn_appr_1_);
  l_appr_pose->addWidget(rbtn_appr_2_);
  l_appr_pose->addWidget(rbtn_appr_3_);  
  layout->addWidget(lb_appr_pose_);
  layout->addLayout(l_appr_pose);

  QHBoxLayout* l_appr_dist = new QHBoxLayout;
  lb_appr_dist_ = new QLabel(QString::fromStdString("displacement"));
  l_appr_dist->addWidget(lb_appr_dist_);
  l_appr_dist->addWidget(slider_);
  l_appr_dist->addWidget(btn_imarker_clear_);
  layout->addLayout(l_appr_dist);

  QHBoxLayout* l_appr_ik = new QHBoxLayout;
  lb_appr_ik_ = new QLabel(QString::fromStdString("select arm configuration"));
  l_appr_ik->addWidget(lb_appr_ik_);
  l_appr_ik->addWidget(combo_box_);
  layout->addLayout(l_appr_ik);

  // Manipulator Motion
  lb_motion_ = new QLabel(QString::fromStdString("Manipulator Motion"));
  lb_motion_->setAlignment(Qt::AlignCenter);
  QHBoxLayout* l_motion_ = new QHBoxLayout;
  // l_motion_->setAlignment(Qt::AlignCenter);
  l_motion_->addWidget(rbtn_motion_1_);
  l_motion_->addWidget(rbtn_motion_2_);
  layout->addWidget(lb_motion_);
  layout->addLayout(l_motion_);

  QHBoxLayout* l_motion_plan_ = new QHBoxLayout;
  QVBoxLayout* l_motion_appr_ = new QVBoxLayout;
  lb_motion_appr_ = new QLabel(QString::fromStdString("Approach"));
  lb_motion_appr_->setAlignment(Qt::AlignCenter);
  l_motion_appr_->addWidget(lb_motion_appr_);
  l_motion_appr_->addWidget(btn_appr_plan_);
  l_motion_appr_->addWidget(btn_appr_execute_);
  QVBoxLayout* l_motion_retr_ = new QVBoxLayout;
  lb_motion_retr_ = new QLabel(QString::fromStdString("Retreat"));
  lb_motion_retr_->setAlignment(Qt::AlignCenter);
  l_motion_retr_->addWidget(lb_motion_retr_);
  l_motion_retr_->addWidget(btn_retr_plan_);
  l_motion_retr_->addWidget(btn_retr_execute_);
  l_motion_plan_->addLayout(l_motion_appr_);
  l_motion_plan_->addLayout(l_motion_retr_);
  layout->addLayout(l_motion_plan_);

  // Move End-Effector
  lb_ee_ = new QLabel(QString::fromStdString("Move End-Effector"));
  lb_ee_->setAlignment(Qt::AlignCenter);
  // QHBoxLayout* l_ee = new QHBoxLayout;
  // QVBoxLayout* l_ee_x = new QVBoxLayout;
  // QVBoxLayout* l_ee_y = new QVBoxLayout;
  // QVBoxLayout* l_ee_z = new QVBoxLayout;
  // l_ee_x->addWidget(btn_move_xp_);
  // l_ee_x->addWidget(btn_move_xm_);
  // l_ee_y->addWidget(btn_move_yp_);
  // l_ee_y->addWidget(btn_move_ym_);
  // l_ee_z->addWidget(btn_move_zp_);
  // l_ee_z->addWidget(btn_move_zm_);
  // l_ee->addLayout(l_ee_x);
  // l_ee->addLayout(l_ee_y);
  // l_ee->addLayout(l_ee_z);
  QHBoxLayout* l_ee_z = new QHBoxLayout;
  l_ee_z->addWidget(btn_move_zp_);
  l_ee_z->addWidget(btn_move_zm_);
  QGridLayout* l_ee = new QGridLayout();
  l_ee->addWidget(btn_move_xp_, 1, 2);
  l_ee->addWidget(btn_move_xm_, 3, 2);
  l_ee->addWidget(btn_move_yp_, 2, 1);
  l_ee->addWidget(btn_move_ym_, 2, 3);
  l_ee->addLayout(l_ee_z, 2, 2);
  layout->addWidget(lb_ee_);
  layout->addLayout(l_ee);
  
  lb_gripper_ = new QLabel(QString::fromStdString("Gripper Action"));
  lb_gripper_->setAlignment(Qt::AlignCenter);
  QHBoxLayout* l_gripper = new QHBoxLayout;  
  l_gripper->addWidget(btn_gripper_open_);
  l_gripper->addWidget(btn_gripper_close_);
  layout->addWidget(lb_gripper_);
  layout->addLayout(l_gripper);

  setLayout(layout);


  //////////////////////////////
  // Initial Setting
  //////////////////////////////

  // btn_mb_cancel_->setEnabled(true);
  // btn_rtabmap_pause_->setEnabled(true);
  // btn_rtabmap_resume_->setEnabled(true);

  // btn_pcl_record_->setEnabled(true);
  // btn_pcl_capture_->setEnabled(true);
  // btn_pcl_clear_->setEnabled(true);

  // btn_pick_approach_plan_->setEnabled(true);
  // btn_pick_approach_execute_->setEnabled(true);
  // btn_pick_retreat_plan_->setEnabled(true);
  // btn_pick_retreat_execute_->setEnabled(true);
  // btn_place_approach_plan_->setEnabled(true);
  // btn_place_approach_execute_->setEnabled(true);
  // btn_place_retreat_plan_->setEnabled(true);
  // btn_place_retreat_execute_->setEnabled(true);

  // btn_move_xp_->setEnabled(true);
  // btn_move_xm_->setEnabled(true);
  // btn_move_yp_->setEnabled(true);
  // btn_move_ym_->setEnabled(true);
  // btn_move_zp_->setEnabled(true);
  // btn_move_zm_->setEnabled(true);

  // btn_gripper_open_->setEnabled(true);
  // btn_gripper_close_->setEnabled(true);

  // btn_imarker_clear_->setEnabled(true);

  rbtn_appr_3_->setChecked(true);
  rbtn_motion_1_->setChecked(true);
  set_all_disabled();

  // goal_reached_flag = 0;
  mb_last = 0;
  m_motion = 0; // 0-pick up, 1-place

}

void MMGuiRviz::cancelMB()
{
  remote_reciever_.publishCancelMB();
}

void MMGuiRviz::pauseRtabmap()
{
  remote_reciever_.callPauseRtabmap();
}

void MMGuiRviz::resumeRtabmap()
{
  remote_reciever_.callResumeRtabmap();
}

void MMGuiRviz::moveGripperOpen()
{
  remote_reciever_.publishGripperOpen();
}

void MMGuiRviz::moveGripperClose()
{
  remote_reciever_.publishGripperClose();
}

void MMGuiRviz::pclRecord()
{
  remote_reciever_.publishPclRecord();
}

void MMGuiRviz::pclCapture()
{
  remote_reciever_.publishPclCapture();
}

void MMGuiRviz::pclClear()
{
  remote_reciever_.publishPclClear();
}

void MMGuiRviz::apprPlan()
{
  if(m_motion == 0) remote_reciever_.publishPickApproachPlan();
  else              remote_reciever_.publishPlaceApproachPlan();
}

void MMGuiRviz::apprExecute()
{
  if(m_motion == 0) remote_reciever_.publishPickApproachExecute();
  else              remote_reciever_.publishPlaceApproachExecute();
}

void MMGuiRviz::retrPlan()
{
  if(m_motion == 0) remote_reciever_.publishPickRetreatPlan();
  else              remote_reciever_.publishPlaceRetreatPlan();
}

void MMGuiRviz::retrExecute()
{
  if(m_motion == 0) remote_reciever_.publishPickRetreatExecute();
  else              remote_reciever_.publishPlaceRetreatExecute();
}

void MMGuiRviz::moveXP()
{
  remote_reciever_.publishMoveXP();
}

void MMGuiRviz::moveXM()
{
  remote_reciever_.publishMoveXM();
}

void MMGuiRviz::moveYP()
{
  remote_reciever_.publishMoveYP();
}

void MMGuiRviz::moveYM()
{
  remote_reciever_.publishMoveYM();
}

void MMGuiRviz::moveZP()
{
  remote_reciever_.publishMoveZP();
}

void MMGuiRviz::moveZM()
{
  remote_reciever_.publishMoveZM();
}

void MMGuiRviz::apprRB1()
{
  remote_reciever_.publishRB(1);
}

void MMGuiRviz::apprRB2()
{
  remote_reciever_.publishRB(2);
}

void MMGuiRviz::apprRB3()
{
  remote_reciever_.publishRB(3);
}

void MMGuiRviz::motionRB1()
{
  m_motion = 0;
}

void MMGuiRviz::motionRB2()
{
  m_motion = 1;
}

void MMGuiRviz::setDistance(int value)
{
  remote_reciever_.publishDistance(value);
}

void MMGuiRviz::set_all_disabled()
{
  lb_mb_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  lb_mb_loc_->setStyleSheet("color:gray;");
  btn_mb_cancel_->setEnabled(false);
  btn_rtabmap_pause_->setEnabled(false);
  btn_rtabmap_resume_->setEnabled(false);  

  lb_pcl_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  btn_pcl_record_->setEnabled(false);
  btn_pcl_capture_->setEnabled(false);
  btn_pcl_clear_->setEnabled(false);

  lb_appr_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  lb_appr_pose_->setStyleSheet("color:gray;");
  lb_appr_dist_->setStyleSheet("color:gray;");
  lb_appr_ik_->setStyleSheet("color:gray;");
  rbtn_appr_1_->setEnabled(false);
  rbtn_appr_2_->setEnabled(false);
  rbtn_appr_3_->setEnabled(false);
  slider_->setEnabled(false);
  btn_imarker_clear_->setEnabled(false);
  combo_box_->setEnabled(false);  

  lb_motion_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  lb_motion_appr_->setStyleSheet("color:gray;");
  lb_motion_retr_->setStyleSheet("color:gray;");
  rbtn_motion_1_->setEnabled(false);
  rbtn_motion_2_->setEnabled(false);
  btn_appr_plan_->setEnabled(false);
  btn_appr_execute_->setEnabled(false);
  btn_retr_plan_->setEnabled(false);
  btn_retr_execute_->setEnabled(false);

  lb_ee_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  btn_move_xp_->setEnabled(false);
  btn_move_xm_->setEnabled(false);
  btn_move_yp_->setEnabled(false);
  btn_move_ym_->setEnabled(false);
  btn_move_zp_->setEnabled(false);
  btn_move_zm_->setEnabled(false);

  lb_gripper_->setStyleSheet("background-color:rgb(186, 189, 182); color:gray;");
  btn_gripper_open_->setEnabled(false);
  btn_gripper_close_->setEnabled(false);
}

void MMGuiRviz::set_all_enabled()
{
  lb_mb_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  lb_mb_loc_->setStyleSheet("color:black;");
  btn_mb_cancel_->setEnabled(true);
  btn_rtabmap_pause_->setEnabled(true);
  btn_rtabmap_resume_->setEnabled(true);  

  lb_pcl_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  btn_pcl_record_->setEnabled(true);
  btn_pcl_capture_->setEnabled(true);
  btn_pcl_clear_->setEnabled(true);

  lb_appr_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  lb_appr_pose_->setStyleSheet("color:black;");
  lb_appr_dist_->setStyleSheet("color:black;");
  lb_appr_ik_->setStyleSheet("color:black;");
  rbtn_appr_1_->setEnabled(true);
  rbtn_appr_2_->setEnabled(true);
  rbtn_appr_3_->setEnabled(true);
  slider_->setEnabled(true);
  btn_imarker_clear_->setEnabled(true);
  combo_box_->setEnabled(true);  

  lb_motion_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  lb_motion_appr_->setStyleSheet("color:black;");
  lb_motion_retr_->setStyleSheet("color:black;");
  rbtn_motion_1_->setEnabled(true);
  rbtn_motion_2_->setEnabled(true);
  btn_appr_plan_->setEnabled(true);
  btn_appr_execute_->setEnabled(true);
  btn_retr_plan_->setEnabled(true);
  btn_retr_execute_->setEnabled(true);

  lb_ee_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  btn_move_xp_->setEnabled(true);
  btn_move_xm_->setEnabled(true);
  btn_move_yp_->setEnabled(true);
  btn_move_ym_->setEnabled(true);
  btn_move_zp_->setEnabled(true);
  btn_move_zm_->setEnabled(true);

  lb_gripper_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
  btn_gripper_open_->setEnabled(true);
  btn_gripper_close_->setEnabled(true);
}

void MMGuiRviz::set_step(int step)
{
  set_all_disabled();
  switch(step) {
    case 1:
      lb_mb_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
      lb_mb_loc_->setStyleSheet("color:black;");
      btn_mb_cancel_->setEnabled(true);
      btn_rtabmap_pause_->setEnabled(true);
      btn_rtabmap_resume_->setEnabled(true);
      break;
    case 2:
      lb_pcl_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
      btn_pcl_record_->setEnabled(true);
      btn_pcl_capture_->setEnabled(true);
      btn_pcl_clear_->setEnabled(true);
      break;
    case 3:
      lb_appr_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
      lb_appr_pose_->setStyleSheet("color:black;");
      lb_appr_dist_->setStyleSheet("color:black;");
      lb_appr_ik_->setStyleSheet("color:black;");
      rbtn_appr_1_->setEnabled(true);
      rbtn_appr_2_->setEnabled(true);
      rbtn_appr_3_->setEnabled(true);
      slider_->setEnabled(true);
      btn_imarker_clear_->setEnabled(true);
      combo_box_->setEnabled(true);
      break;
    case 4:
      lb_motion_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
      lb_motion_appr_->setStyleSheet("color:black;");
      lb_motion_retr_->setStyleSheet("color:black;");
      rbtn_motion_1_->setEnabled(true);
      rbtn_motion_2_->setEnabled(true);
      btn_appr_plan_->setEnabled(true);
      btn_appr_execute_->setEnabled(true);
      btn_retr_plan_->setEnabled(true);
      btn_retr_execute_->setEnabled(true);
      break;
    case 5:
      lb_ee_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
      btn_move_xp_->setEnabled(true);
      btn_move_xm_->setEnabled(true);
      btn_move_yp_->setEnabled(true);
      btn_move_ym_->setEnabled(true);
      btn_move_zp_->setEnabled(true);
      btn_move_zm_->setEnabled(true);
      lb_gripper_->setStyleSheet("background-color:rgb(186, 189, 182); color:black;");
      btn_gripper_open_->setEnabled(true);
      btn_gripper_close_->setEnabled(true);
      break;
    case 6:
      set_all_enabled();
      break;
    default:
      break;
  }
}

void MMGuiRviz::updateText()
{
  text_browser_->setText(QString::fromStdString(remote_reciever_.get_instruction()));
}

// void MMGuiRviz::updateMBResult()
// {
//   int result_status = remote_reciever_.get_mb_result();
//   if (result_status == 3 && goal_reached_flag == 0){
//     QMessageBox::information(this, "MoveBase", "Goal Reached");
//     goal_reached_flag = 1;
//   }
//   // printf("move base result status is %d\n", result_status);
// }

void MMGuiRviz::updateMBStatus()
{
  int mb_current = remote_reciever_.get_mb_status();

  // message when the goal is reached
  if (mb_last == 1 && mb_current == 3)
  {
    QMessageBox::information(this, "MoveBase", "Goal Reached");
  }

  // printf(mb_current);

  switch(mb_current) {
    case 1:
      // The goal is currently being processed by the action server
      mb_last = mb_current;
      break;
    case 2:
      // The goal received a cancel request after it started executing
      //   and has since completed its execution (Terminal State)
      mb_last = mb_current;
      break;
    case 3:
      // The goal was achieved successfully by the action server (Terminal State)
      mb_last = mb_current;
      break;
    case 4:
      // The goal was aborted during execution by the action server due
      //  to some failure (Terminal State)
      mb_last = mb_current;
      break;
    case 6:
      // The goal received a cancel request after it started executing
      //    and has not yet completed execution
      mb_last = mb_current;
      break;
  }
}

void MMGuiRviz::updateMMStep()
{
  int mm_current = remote_reciever_.get_mm_step();
  
  if(mm_last != mm_current) set_step(mm_current);

  mm_last = mm_current;
}


void MMGuiRviz::clearIMarker()
{
  rbtn_appr_3_->setChecked(true);
  slider_->setValue(10);
  remote_reciever_.publishClearIMarker();
}

void MMGuiRviz::selectSolution(int value)
{
  remote_reciever_.publishSolution(value-1);
}

void MMGuiRviz::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void MMGuiRviz::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace mm_gui_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mm_gui_rviz::MMGuiRviz, rviz::Panel)
