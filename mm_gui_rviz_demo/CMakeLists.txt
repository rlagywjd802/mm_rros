cmake_minimum_required(VERSION 2.8.3)
project(mm_gui_rviz_demo)

add_definitions(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warnings
add_definitions(-W -Wall -Wextra
  #-Wcast-qual
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  graph_msgs
  roscpp
  rostest
  std_msgs
  tf_conversions
  trajectory_msgs
  visualization_msgs
  roslint
  eigen_stl_containers
  rviz
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system)
# Qt 4 or 5
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()
## Prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    visualization_msgs
    graph_msgs
    std_msgs
    tf_conversions
    trajectory_msgs
    sensor_msgs
    roscpp
  INCLUDE_DIRS
    include
)

include_directories(SYSTEM include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

###########
## Build ##
###########

set(SOURCE_FILES
  src/${PROJECT_NAME}.cpp
)

# Rviz GUI library
add_library(${PROJECT_NAME} ${SOURCE_FILES})
# add_library(${PROJECT_NAME} ${SOURCE_FILES} src/remote_control.cpp)
target_link_libraries(${PROJECT_NAME} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

##########
## TEST ##
##########

#############
## Install ##
#############

# Install libraries
install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install xml plugin config
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)