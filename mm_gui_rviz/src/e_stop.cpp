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
#include "e_stop.h"

// C++
#include <sstream>

namespace mm_gui_rviz
{
EStop::EStop() = default;

EStop::~EStop() = default;

void EStop::onInitialize()
{
  move_tool_.initialize(context_);
  printf("EStop initialized\n");
}

void EStop::activate()
{
  remote_reciever_.publishEStop(true);
  printf("EStop activate\n");
}

void EStop::deactivate()
{
  remote_reciever_.publishEStop(false);
  printf("EStop deactivate\n");
}

// int EStop::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
// {
//   return 1;
// }

// int EStop::processMouseEvent(rviz::ViewportMouseEvent& event)
// {
//   int flags = 0;
//   move_tool_.processMouseEvent(event);
//   setCursor(move_tool_.getCursor());

//   return flags;
// }

}  // namespace mm_gui_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mm_gui_rviz::EStop, rviz::Tool)
