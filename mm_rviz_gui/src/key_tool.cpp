/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreRay.h>
#include <OgreVector3.h>

// Rviz
#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/view_controller.h>
#include <rviz/viewport_mouse_event.h>
#include <geometry_msgs/Twist.h>

// this package
#include "key_tool.h"

// C++
#include <sstream>

namespace mm_rviz_gui
{
KeyTool::KeyTool() = default;

KeyTool::~KeyTool() = default;

void KeyTool::onInitialize()
{
  move_tool_.initialize(context_);
}

void KeyTool::activate()
{
}

void KeyTool::deactivate()
{
}

int KeyTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  int x = 0;
  int th = 0;
  // move forward / backward
  switch (event->key())
  {
    case Qt::Key_1:
      x = -1;
      th = -1;
      printf("x=%d, th=%d\n", x, th);
      break;
    case Qt::Key_2:
      x = -1;
      th = 0;
      break;
    case Qt::Key_3:
      x = -1;
      th = 1;
      break;
    case Qt::Key_4:
      x = 0;
      th = 1;
      break;
    case Qt::Key_5:
      break;
    case Qt::Key_6:
      x = 0;
      th = -1;
      break;
    case Qt::Key_7:
      x = 1;
      th = 1;
      break;
    case Qt::Key_8:
      x = 1;
      th = 0;
      break;
    case Qt::Key_9:
      x = 1;
      th = -1;
      break;
    default:
      return move_tool_.processKeyEvent(event, panel);
  }

  // twist
  geometry_msgs::Twist twist;
  twist.linear.x = x*speed_; 
  twist.linear.y = 0; 
  twist.linear.z = 0;
  twist.angular.x = 0; 
  twist.angular.y = 0; 
  twist.angular.z = th*turn_;
  printf("x=%d, th=%d\n", x, th);

  remote_reciever_.publishCmdVel(twist);

  return 1;
}

int KeyTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = 0;

  move_tool_.processMouseEvent(event);
  setCursor(move_tool_.getCursor());

  return flags;
}

}  // namespace mm_rviz_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mm_rviz_gui::KeyTool, rviz::Tool)
