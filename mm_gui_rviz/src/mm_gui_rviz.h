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

#ifndef MM_GUI_RVIZ__MM_GUI_RVIZ_H
#define MM_GUI_RVIZ__MM_GUI_RVIZ_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QComboBox>

#include <QRadioButton>
#include <QTextBrowser>
#include <QSlider>
#include <QTableWidget>
#include <QStringList>
#include <QHeaderView>
#include <QLabel>

#include <mm_gui_rviz/remote_reciever.h>

class QLineEdit;
class QSpinBox;
class QVBoxLayout;
class QHBoxLayout;
class QFrame;

namespace mm_gui_rviz
{
class MMGuiRviz : public rviz::Panel
{
  Q_OBJECT
public:
  explicit MMGuiRviz(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  
  void updateText();

  // void updateIKCost();

  // void updateMBResult();
  void updateMBStatus();

  void updateMMStep();


protected Q_SLOTS:

  void cancelMB();

  void pauseRtabmap();

  void resumeRtabmap();

  void apprPlan();

  void apprExecute();

  void retrPlan();

  void retrExecute();
  
  void moveGripperOpen();
  
  void moveGripperClose();

  void pclRecord();

  void pclCapture();

  void pclClear();

  void emergencyStop();

  void moveXP();

  void moveXM();

  void moveYP();

  void moveYM();

  void moveZP();

  void moveZM();

  void apprRB1();

  void apprRB2();

  void apprRB3();

  void motionRB1();

  void motionRB2();

  void setDistance(int);

  void clearIMarker();

  void selectSolution(int);

  void set_all_disabled();

  void set_all_enabled();

  void set_step(int);

protected:
  QVBoxLayout* layout;

  QPushButton* btn_mb_cancel_;

  QPushButton* btn_rtabmap_pause_;
  QPushButton* btn_rtabmap_resume_;
  
  QPushButton* btn_gripper_open_;
  QPushButton* btn_gripper_close_;
  
  QPushButton* btn_pcl_record_;
  QPushButton* btn_pcl_capture_;
  QPushButton* btn_pcl_clear_;

  QPushButton* btn_imarker_clear_;

  QPushButton* btn_appr_plan_;
  QPushButton* btn_appr_execute_;
  QPushButton* btn_retr_plan_;
  QPushButton* btn_retr_execute_;

  QPushButton* btn_move_xp_;
  QPushButton* btn_move_xm_;
  QPushButton* btn_move_yp_;
  QPushButton* btn_move_ym_;
  QPushButton* btn_move_zp_;
  QPushButton* btn_move_zm_;

  QRadioButton* rbtn_appr_1_;
  QRadioButton* rbtn_appr_2_;
  QRadioButton* rbtn_appr_3_;

  QRadioButton* rbtn_motion_1_;
  QRadioButton* rbtn_motion_2_;

  QSlider* slider_;

  QTimer* timer_;
  QTextBrowser* text_browser_;

  QComboBox *combo_box_;

  QLabel* lb_mb_;
  QLabel* lb_mb_loc_;
  QLabel* lb_pcl_;
  QLabel* lb_appr_;
  QLabel* lb_appr_pose_;
  QLabel* lb_appr_dist_;
  QLabel* lb_appr_ik_;
  QLabel* lb_motion_;
  QLabel* lb_motion_appr_;
  QLabel* lb_motion_retr_;
  QLabel* lb_ee_;
  QLabel* lb_gripper_;
  
  RemoteReciever remote_reciever_;

  // int goal_reached_flag; 
  int mb_last;
  int mm_last;
  int m_motion;

};

}  // end namespace mm_gui_rviz

#endif  // MM_GUI_RVIZ__MM_GUI_RVIZ_H