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
  btn_rtabmap_pause_ = new QPushButton(this);
  btn_rtabmap_pause_->setText("Pause");
  connect(btn_rtabmap_pause_, SIGNAL(clicked()), this, SLOT(pauseRtabmap()));

  // Create a push button
  btn_rtabmap_resume_ = new QPushButton(this);
  btn_rtabmap_resume_->setText("Resume");
  connect(btn_rtabmap_resume_, SIGNAL(clicked()), this, SLOT(resumeRtabmap()));

  // Create a push button
  btn_gripper_open_ = new QPushButton(this);
  btn_gripper_open_->setText("Open");
  connect(btn_gripper_open_, SIGNAL(clicked()), this, SLOT(moveGripperOpen()));

  // Create a push button
  btn_gripper_close_ = new QPushButton(this);
  btn_gripper_close_->setText("Close");
  connect(btn_gripper_close_, SIGNAL(clicked()), this, SLOT(moveGripperClose()));

  // Create a push button
  btn_pcl_record_ = new QPushButton(this);
  btn_pcl_record_->setText("Record");
  connect(btn_pcl_record_, SIGNAL(clicked()), this, SLOT(pclRecord()));

  // Create a push button
  btn_pcl_capture_ = new QPushButton(this);
  btn_pcl_capture_->setText("Capture");
  connect(btn_pcl_capture_, SIGNAL(clicked()), this, SLOT(pclCapture()));

  // Create a push button
  btn_pcl_clear_ = new QPushButton(this);
  btn_pcl_clear_->setText("Clear");
  connect(btn_pcl_clear_, SIGNAL(clicked()), this, SLOT(pclClear()));

  // Create a push button
  btn_pick_approach_plan_ = new QPushButton(this);
  btn_pick_approach_plan_->setText("Plan");
  connect(btn_pick_approach_plan_, SIGNAL(clicked()), this, SLOT(pickApproachPlan()));

  // Create a push button
  btn_pick_approach_execute_ = new QPushButton(this);
  btn_pick_approach_execute_->setText("Execute");
  connect(btn_pick_approach_execute_, SIGNAL(clicked()), this, SLOT(pickApproachExecute()));

  // Create a push button
  btn_pick_retreat_plan_ = new QPushButton(this);
  btn_pick_retreat_plan_->setText("Plan");
  connect(btn_pick_retreat_plan_, SIGNAL(clicked()), this, SLOT(pickRetreatPlan()));

  // Create a push button
  btn_pick_retreat_execute_ = new QPushButton(this);
  btn_pick_retreat_execute_->setText("Execute");
  connect(btn_pick_retreat_execute_, SIGNAL(clicked()), this, SLOT(pickRetreatExecute()));

  // Create a push button
  btn_place_approach_plan_ = new QPushButton(this);
  btn_place_approach_plan_->setText("Plan");
  connect(btn_place_approach_plan_, SIGNAL(clicked()), this, SLOT(placeApproachPlan()));

  // Create a push button
  btn_place_approach_execute_ = new QPushButton(this);
  btn_place_approach_execute_->setText("Execute");
  connect(btn_place_approach_execute_, SIGNAL(clicked()), this, SLOT(placeApproachExecute()));

  // Create a push button
  btn_place_retreat_plan_ = new QPushButton(this);
  btn_place_retreat_plan_->setText("Plan");
  connect(btn_place_retreat_plan_, SIGNAL(clicked()), this, SLOT(placeRetreatPlan()));

  // Create a push button
  btn_place_retreat_execute_ = new QPushButton(this);
  btn_place_retreat_execute_->setText("Execute");
  connect(btn_place_retreat_execute_, SIGNAL(clicked()), this, SLOT(placeRetreatExecute()));

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

  // Create a radio button
  rbtn_1_ = new QRadioButton("rx(red)", this);
  rbtn_2_ = new QRadioButton("ry(green)", this);
  rbtn_3_ = new QRadioButton("rz(blue)", this);
  connect(rbtn_1_, SIGNAL(clicked()), this, SLOT(testRB1()));  
  connect(rbtn_2_, SIGNAL(clicked()), this, SLOT(testRB2()));  
  connect(rbtn_3_, SIGNAL(clicked()), this, SLOT(testRB3()));
  
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

  // Create a text browser
  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateText()));
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateIKCost()));
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateMBResult()));
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

  // addTitle("Instruction Panel");
  layout->addWidget(text_browser_);

  addTitle("Rtabmap Localization");
  rtabmap_layout = new QHBoxLayout;
  rtabmap_layout->addWidget(btn_rtabmap_pause_);
  rtabmap_layout->addWidget(btn_rtabmap_resume_);
  layout->addLayout(rtabmap_layout);  

  addTitle("Capture Point Cloud Data");
  pcl_layout = new QHBoxLayout;
  pcl_layout->addWidget(btn_pcl_record_);
  pcl_layout->addWidget(btn_pcl_capture_);
  pcl_layout->addWidget(btn_pcl_clear_);
  layout->addLayout(pcl_layout);

  addTitle("Set Pre-Grasp Pose");
  sl_layout = new QHBoxLayout;
  sl_layout->addWidget(slider_);
  layout->addLayout(sl_layout);

  rb_layout = new QHBoxLayout;
  rb_layout->addWidget(rbtn_1_);
  rb_layout->addWidget(rbtn_2_);
  rb_layout->addWidget(rbtn_3_);
  layout->addLayout(rb_layout);

  layout->addWidget(btn_imarker_clear_);

  addTitle("Select IK Solution");
  layout->addWidget(combo_box_);

  addTitle("Pick");
  pick_layout = new QHBoxLayout;
  pick1_layout = new QVBoxLayout;
  pick2_layout = new QVBoxLayout;

  addTitlePick1("Approach");
  pick1_layout->addWidget(btn_pick_approach_plan_);
  pick1_layout->addWidget(btn_pick_approach_execute_);

  addTitlePick2("Retreat");
  pick2_layout->addWidget(btn_pick_retreat_plan_);
  pick2_layout->addWidget(btn_pick_retreat_execute_);

  pick_layout->addLayout(pick1_layout);
  pick_layout->addLayout(pick2_layout);
  layout->addLayout(pick_layout);

  addTitle("Place");
  place_layout = new QHBoxLayout;
  place1_layout = new QVBoxLayout;
  place2_layout = new QVBoxLayout;

  addTitlePlace1("Approach");
  place1_layout->addWidget(btn_place_approach_plan_);
  place1_layout->addWidget(btn_place_approach_execute_);

  addTitlePlace2("Retreat");
  place2_layout->addWidget(btn_place_retreat_plan_);
  place2_layout->addWidget(btn_place_retreat_execute_);

  place_layout->addLayout(place1_layout);
  place_layout->addLayout(place2_layout);
  layout->addLayout(place_layout);

  addTitle("UR5 actions");  
  sub_layout = new QHBoxLayout;
  subx_layout = new QVBoxLayout;
  suby_layout = new QVBoxLayout;
  subz_layout = new QVBoxLayout;

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

  addTitle("Gripper actions");
  gripper_layout = new QHBoxLayout;  
  gripper_layout->addWidget(btn_gripper_open_);
  gripper_layout->addWidget(btn_gripper_close_);
  layout->addLayout(gripper_layout);

  setLayout(layout);


  //////////////////////////////
  // Initial Setting
  //////////////////////////////

  btn_rtabmap_pause_->setEnabled(true);
  btn_rtabmap_resume_->setEnabled(true);

  btn_pcl_record_->setEnabled(true);
  btn_pcl_capture_->setEnabled(true);
  btn_pcl_clear_->setEnabled(true);

  btn_pick_approach_plan_->setEnabled(true);
  btn_pick_approach_execute_->setEnabled(true);
  btn_pick_retreat_plan_->setEnabled(true);
  btn_pick_retreat_execute_->setEnabled(true);
  btn_place_approach_plan_->setEnabled(true);
  btn_place_approach_execute_->setEnabled(true);
  btn_place_retreat_plan_->setEnabled(true);
  btn_place_retreat_execute_->setEnabled(true);

  btn_move_xp_->setEnabled(true);
  btn_move_xm_->setEnabled(true);
  btn_move_yp_->setEnabled(true);
  btn_move_ym_->setEnabled(true);
  btn_move_zp_->setEnabled(true);
  btn_move_zm_->setEnabled(true);

  btn_gripper_open_->setEnabled(true);
  btn_gripper_close_->setEnabled(true);

  rbtn_3_->setChecked(true);

  btn_imarker_clear_->setEnabled(true);

  goal_reached_flag = 0;

}

void MMGuiRviz::addLabelX()
{
  QLabel* label = new QLabel(QString::fromStdString("x"));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  subx_layout->addWidget(label);
}

void MMGuiRviz::addLabelY()
{
  QLabel* label = new QLabel(QString::fromStdString("y"));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  suby_layout->addWidget(label);
}

void MMGuiRviz::addLabelZ()
{
  QLabel* label = new QLabel(QString::fromStdString("z"));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  subz_layout->addWidget(label);
}

void MMGuiRviz::addTitlePick1(std::string str)
{
  QLabel* label = new QLabel(QString::fromStdString(str));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  pick1_layout->addWidget(label);
}

void MMGuiRviz::addTitlePick2(std::string str)
{
  QLabel* label = new QLabel(QString::fromStdString(str));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  pick2_layout->addWidget(label);
}

void MMGuiRviz::addTitlePlace1(std::string str)
{
  QLabel* label = new QLabel(QString::fromStdString(str));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  place1_layout->addWidget(label);
}

void MMGuiRviz::addTitlePlace2(std::string str)
{
  QLabel* label = new QLabel(QString::fromStdString(str));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  place2_layout->addWidget(label);
}

void MMGuiRviz::addTitle(std::string str)
{
  QLabel* label = new QLabel(QString::fromStdString(str));
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel { color : black; }");
  layout->addWidget(label);
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

void MMGuiRviz::pickApproachPlan()
{
  remote_reciever_.publishPickApproachPlan();
}

void MMGuiRviz::pickApproachExecute()
{
  remote_reciever_.publishPickApproachExecute();
}

void MMGuiRviz::pickRetreatPlan()
{
  remote_reciever_.publishPickRetreatPlan();
}

void MMGuiRviz::pickRetreatExecute()
{
  remote_reciever_.publishPickRetreatExecute();
}

void MMGuiRviz::placeApproachPlan()
{
  remote_reciever_.publishPlaceApproachPlan();
}

void MMGuiRviz::placeApproachExecute()
{
  remote_reciever_.publishPlaceApproachExecute();
}

void MMGuiRviz::placeRetreatPlan()
{
  remote_reciever_.publishPlaceRetreatPlan();
}

void MMGuiRviz::placeRetreatExecute()
{
  remote_reciever_.publishPlaceRetreatExecute();
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

void MMGuiRviz::testRB1()
{
  remote_reciever_.publishRB(1);
}

void MMGuiRviz::testRB2()
{
  remote_reciever_.publishRB(2);
}

void MMGuiRviz::testRB3()
{
  remote_reciever_.publishRB(3);
}

void MMGuiRviz::setDistance(int value)
{
  remote_reciever_.publishDistance(value);
}

void MMGuiRviz::updateText()
{
  text_browser_->setText(QString::fromStdString(remote_reciever_.get_instruction()));
}

void MMGuiRviz::updateMBResult()
{
  int result_status = remote_reciever_.get_mb_result();
  if (result_status == 3 && goal_reached_flag == 0){
    QMessageBox::information(this, "MoveBase", "Goal Reached");
    goal_reached_flag = 1;
  }
  // printf("move base result status is %d\n", result_status);
}


void MMGuiRviz::clearIMarker()
{
  rbtn_3_->setChecked(true);
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
