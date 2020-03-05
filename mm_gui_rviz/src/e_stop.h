#ifndef MM_GUI_RVIZ_EMERGENCY_STOP_H
#define MM_GUI_RVIZ_EMERGENCY_STOP_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <rviz/tool.h>
#include <rviz/default_plugin/tools/move_tool.h>

#include <QCursor>
#include <QObject>
#endif

#include <mm_gui_rviz/remote_reciever.h>

namespace mm_gui_rviz
{
class EStop : public rviz::Tool
{
  Q_OBJECT
public:
  EStop();
  virtual ~EStop();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  // virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);
  // virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

public Q_SLOTS:

protected:
  rviz::MoveTool move_tool_;
  RemoteReciever remote_reciever_;
};
}  // namespace mm_gui_rviz

#endif
