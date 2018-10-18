cmake_minimum_required(VERSION 2.8.3)
project(bebop_vicon_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation)

  add_service_files(
    FILES
    FlyTo.srv
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

catkin_package()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/teleop_twist_keyboard.py
   src/bebop_controller.py
   src/bebop_geofence.py
   src/update_goal_keyboard.py
   src/bebop_api_server.py
   src/bebop_api_client.py
   src/autonomous_landing.py
   src/play_sounds.py

   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
