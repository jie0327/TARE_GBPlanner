cmake_minimum_required(VERSION 3.0.2)
project(tare_gbplanner_hybrid)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  planner_msgs
  pluginlib
  roscpp
  rospy
  rviz
  sensor_msgs
  std_msgs
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs planner_msgs pluginlib roscpp rospy rviz sensor_msgs std_msgs
  LIBRARIES tare_gbplanner_hybrid_rviz_panel
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(tare_gbplanner_hybrid_rviz_panel
  include/tare_gbplanner_hybrid/exploration_control_panel.h
  src/exploration_control_panel.cpp
)
target_link_libraries(tare_gbplanner_hybrid_rviz_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

catkin_install_python(PROGRAMS
  scripts/hybrid_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS tare_gbplanner_hybrid_rviz_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_adaptive_policy.py)
  catkin_add_nosetests(test/test_launch_contract.py)
endif()
