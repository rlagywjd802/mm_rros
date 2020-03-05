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

namespace mm_gui_rviz
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

// int KeyTool::processMouseEvent(rviz::ViewportMouseEvent& event)
// {
//   int flags = 0;

//   move_tool_.processMouseEvent(event);
//   setCursor(move_tool_.getCursor());

//   return flags;
// }

}  // namespace mm_gui_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mm_gui_rviz::KeyTool, rviz::Tool)
