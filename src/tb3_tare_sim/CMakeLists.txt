cmake_minimum_required(VERSION 3.0.2)
project(tb3_tare_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  pcl_ros
  pcl_conversions
  tf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    message_filters
    pcl_ros
    pcl_conversions
    tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tb3_state_scan_adapter src/tb3_state_scan_adapter.cpp)
target_link_libraries(tb3_state_scan_adapter ${catkin_LIBRARIES})
add_dependencies(tb3_state_scan_adapter ${catkin_EXPORTED_TARGETS})

add_executable(fast_lio_tare_adapter src/fast_lio_tare_adapter.cpp)
target_link_libraries(fast_lio_tare_adapter ${catkin_LIBRARIES})
add_dependencies(fast_lio_tare_adapter ${catkin_EXPORTED_TARGETS})

install(TARGETS tb3_state_scan_adapter fast_lio_tare_adapter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/twist_stamped_to_twist.py
  scripts/twist_watchdog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS
    test/test_fast_lio_adapter_gate.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  add_rostest(test/fast_lio_adapter_gate.test)
  catkin_add_nosetests(test/test_tb3_launch_contract.py)
endif()
