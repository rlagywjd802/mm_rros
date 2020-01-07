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

#ifndef MM_GUI_RVIZ_DEMO__MM_GUI_RVIZ_DEMO_H
#define MM_GUI_RVIZ_DEMO__MM_GUI_RVIZ_DEMO_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QComboBox>

#include <mm_gui_rviz_demo/remote_reciever.h>

class QLineEdit;
class QSpinBox;
class QVBoxLayout;
class QHBoxLayout;
class QFrame;

namespace mm_gui_rviz_demo
{
class MMGuiRvizDemo : public rviz::Panel
{
  Q_OBJECT
public:
  explicit MMGuiRvizDemo(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:

  void addLabelX();

  void addLabelY();

  void addLabelZ();

  void addTitle(std::string str);

  void moveArmPlan();

  void moveArmExecute();

  void moveArmStop();

  void moveGripperOpen();
  
  void moveGripperClose();

  void pclCapture();

  void pclClear();

  void approachArmPlan();

  void approachArmExcute();

  void emergencyStop();

  void moveXP();

  void moveXM();

  void moveYP();

  void moveYM();

  void moveZP();

  void moveZM();


protected:
  QVBoxLayout* layout;
  QHBoxLayout* sub_layout;
  QVBoxLayout* subx_layout;
  QVBoxLayout* suby_layout;
  QVBoxLayout* subz_layout;
  
  QPushButton* btn_gripper_open_;
  QPushButton* btn_gripper_close_;
  QPushButton* btn_pcl_capture_;
  QPushButton* btn_pcl_clear_;
  QPushButton* btn_approach_plan_;
  QPushButton* btn_approach_execute_;
  QPushButton* btn_emergency_stop_;

  QPushButton* btn_move_xp_;
  QPushButton* btn_move_xm_;
  QPushButton* btn_move_yp_;
  QPushButton* btn_move_ym_;
  QPushButton* btn_move_zp_;
  QPushButton* btn_move_zm_;

  QFrame* line;
  RemoteReciever remote_reciever_;
};

}  // end namespace mm_gui_rviz_demo

#endif  // MM_GUI_RVIZ_DEMO__MM_GUI_RVIZ_DEMO_H