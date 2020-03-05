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

protected Q_SLOTS:

  void addLabelX();

  void addLabelY();

  void addLabelZ();

  void addTitle(std::string str);

  void addTitlePick1(std::string str);

  void addTitlePick2(std::string str);

  void addTitlePlace1(std::string str);

  void addTitlePlace2(std::string str);

  void pauseRtabmap();

  void resumeRtabmap();

  void pickApproachPlan();

  void pickApproachExecute();

  void pickRetreatPlan();

  void pickRetreatExecute();
  
  void placeApproachPlan();

  void placeApproachExecute();

  void placeRetreatPlan();

  void placeRetreatExecute();

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

  void testRB1();

  void testRB2();

  void testRB3();

  void setDistance(int);

  void clearIMarker();

  void selectSolution(int);

protected:
  QVBoxLayout* layout;

  QHBoxLayout* rtabmap_layout;
  QHBoxLayout* pcl_layout;

  QHBoxLayout* sub_layout;
  QVBoxLayout* subx_layout;
  QVBoxLayout* suby_layout;
  QVBoxLayout* subz_layout;

  QHBoxLayout* pick_layout;
  QVBoxLayout* pick1_layout;
  QVBoxLayout* pick2_layout;

  QHBoxLayout* place_layout;
  QVBoxLayout* place1_layout;
  QVBoxLayout* place2_layout;

  QHBoxLayout* gripper_layout;

  QPushButton* btn_rtabmap_pause_;
  QPushButton* btn_rtabmap_resume_;
  
  QPushButton* btn_gripper_open_;
  QPushButton* btn_gripper_close_;
  
  QPushButton* btn_pcl_record_;
  QPushButton* btn_pcl_capture_;
  QPushButton* btn_pcl_clear_;

  QPushButton* btn_pick_approach_plan_;
  QPushButton* btn_pick_approach_execute_;
  QPushButton* btn_pick_retreat_plan_;
  QPushButton* btn_pick_retreat_execute_;

  QPushButton* btn_place_approach_plan_;
  QPushButton* btn_place_approach_execute_;
  QPushButton* btn_place_retreat_plan_;
  QPushButton* btn_place_retreat_execute_;

  QPushButton* btn_emergency_stop_;

  QPushButton* btn_move_xp_;
  QPushButton* btn_move_xm_;
  QPushButton* btn_move_yp_;
  QPushButton* btn_move_ym_;
  QPushButton* btn_move_zp_;
  QPushButton* btn_move_zm_;

  QHBoxLayout* rb_layout;
  QRadioButton* rbtn_1_;
  QRadioButton* rbtn_2_;
  QRadioButton* rbtn_3_;

  QHBoxLayout* sl_layout;
  QSlider* slider_;

  QPushButton* btn_imarker_clear_;

  QTimer* timer_;
  QTextBrowser* text_browser_;

  QComboBox *combo_box_;

  QFrame* line;
  RemoteReciever remote_reciever_;  

};

}  // end namespace mm_gui_rviz

#endif  // MM_GUI_RVIZ__MM_GUI_RVIZ_H