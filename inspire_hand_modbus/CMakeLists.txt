cmake_minimum_required(VERSION 3.10)

# set the project name
project(inspire_hand_modbus) 

# find the libmodbus library
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  message_generation
  tf
  std_msgs
)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include the header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${MODBUS_INCLUDE_DIRS}
)

# add the message files
add_message_files(
  FILES
  get_force_act_1.msg
  get_angle_act_1.msg
  get_touch_act_1.msg
  
  set_angle_1.msg
  set_force_1.msg
  set_speed_1.msg  
)

# add the service files
add_service_files(
  FILES
  get_current.srv
  get_error.srv
  get_pos_set.srv
  get_pos_act.srv
  get_force_act.srv
  get_force_set.srv
  get_angle_set.srv
  get_angle_act.srv
  get_temp.srv
  get_status.srv
  
  set_id.srv
  set_redu_ratio.srv
  set_pos.srv
  set_clear_error.srv
  set_reset_para.srv
  set_gesture_no.srv
  set_default_speed.srv
  set_speed.srv
  set_angle.srv
  set_save_flash.srv
  set_force.srv
  set_default_force.srv
  set_force_clb.srv
  set_current_limit.srv
)

# specify the message generation
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

# install the Python scripts
catkin_install_python(PROGRAMS
  scripts/handcontrol_topic_publisher_modbus.py
  scripts/handcontrol_topic_subscriber_modbus.py
  scripts/inspire_hand_modbus_topic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# add the executable files
add_executable(hand_modbus_control 
  src/main.cpp
  src/hand_control.cpp
)

# add the hand_control_client executable file
add_executable(hand_control_client_modbus src/hand_control_client_modbus.cpp)

# add the dependencies
add_dependencies(hand_modbus_control ${catkin_EXPORTED_TARGETS} inspire_hand_modbus_gencpp)
add_dependencies(hand_control_client_modbus ${catkin_EXPORTED_TARGETS} inspire_hand_modbus_gencpp)

# link the libraries
target_link_libraries(hand_modbus_control
  ${catkin_LIBRARIES}
  ${MODBUS_LIBRARIES}
)

target_link_libraries(hand_control_client_modbus
  ${catkin_LIBRARIES}
  ${MODBUS_LIBRARIES}
)

