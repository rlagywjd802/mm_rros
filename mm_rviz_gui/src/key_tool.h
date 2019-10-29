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

#ifndef MM_RVIZ_GUI_KEY_TOOL_H
#define MM_RVIZ_GUI_KEY_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <rviz/tool.h>
#include <rviz/default_plugin/tools/move_tool.h>

#include <QCursor>
#include <QObject>
#endif

#include <mm_rviz_gui/remote_reciever.h>

namespace mm_rviz_gui
{
class KeyTool : public rviz::Tool
{
  Q_OBJECT
public:
  KeyTool();
  virtual ~KeyTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

public Q_SLOTS:

protected:
  rviz::MoveTool move_tool_;
  RemoteReciever remote_reciever_;
  float speed_ = 0.1;
  float turn_ = 0.2;

};
}  // namespace mm_rviz_gui

#endif