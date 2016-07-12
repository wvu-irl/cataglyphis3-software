cmake_minimum_required(VERSION 2.8.3)
project(hsm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  messages
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	HSM_Detection.msg
	HSM_Action.msg
	UserInputInitStartup.msg
	UserInputReboot.msg
	UserInputLostLocation.msg
	UserInputLostNorth.msg
	UserInputLostNextState.msg
	UserInputLostWaypointIndex.msg
	UserInputLostSampleProps.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
#   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES HSM_Detector user_input_mp_act_class user_input_exec_act_class user_input_nav_act_class HSM_Heartbeat
  CATKIN_DEPENDS roscpp rospy message_generation messages
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_definitions("-std=gnu++11")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/hsm
  ~/cataglyphis_ws/src/robot_control/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 #add_library(HSM_Detector src/HSM_Detector.cpp)
 add_library(Comm_Monitor_class src/Comm_Monitor_class.cpp)
 add_library(Escape_Monitor_class src/Escape_Monitor_class.cpp)
 add_library(Counter src/Counter.cpp)
 #add_library(Timer src/Timer.cpp)
 #add_library(user_input_mp_act_class src/user_input_mp_act_class.cpp)
 #add_library(user_input_exec_act_class src/user_input_exec_act_class.cpp)
 add_library(user_input_nav_act_class src/user_input_nav_act_class.cpp)
 #add_library(HSM_Heartbeat src/HSM_Heartbeat.cpp)
 #add_library(HSM_Heartbeat_Monitor_class src/HSM_Heartbeat_Monitor_class.cpp)
 add_library(robot_pose_monitor src/robot_pose_monitor.cpp)

## Declare a cpp executable
 #add_executable(TEST_DRIVER_HSM_Detector src/TEST_DRIVER_HSM_Detector.cpp src/HSM_Detector.cpp)
 #add_executable(TEST_DRIVER_Counter src/TEST_DRIVER_Counter.cpp src/Counter.cpp)
 #add_executable(TEST_DRIVER_Timer src/TEST_DRIVER_Timer.cpp src/Timer.cpp)
 add_executable(HSM_Master_Executive src/HSM_Master_Executive_node.cpp)
 add_executable(user_input_node src/user_input_node.cpp)
 #add_executable(info_node src/info_node.cpp)
 #add_executable(HSM_NB1_Executive src/HSM_NB1_Executive_node.cpp src/HSM_NB1_Monitor_class src/Counter)
 #add_executable(HSM_HB_Executive src/HSM_HB_Executive_node.cpp src/HSM_Heartbeat_Monitor_class src/Counter)
 
 #add_executable(HSM_TEST_node_1 src/HSM_TEST_node_1.cpp src/HSM_Heartbeat.cpp src/HSM_Detector.cpp src/Counter.cpp)
 #add_executable(HSM_TEST_node_2 src/HSM_TEST_node_2.cpp src/HSM_Heartbeat.cpp src/HSM_Detector.cpp src/Counter.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 #add_dependencies(hsm_node hsm_generate_messages_cpp)
 #add_dependencies(TEST_DRIVER_HSM_Detector  hsm_generate_messages_cpp)
 #add_dependencies(TEST_DRIVER_HSM_Detector ${catkin_EXPORTED_TARGETS})
 add_dependencies(HSM_Master_Executive ${catkin_EXPORTED_TARGETS})
 add_dependencies(HSM_Master_Executive hsm_generate_messages_cpp)
 add_dependencies(user_input_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(user_input_node hsm_generate_messages_cpp robot_control_generate_messages_cpp navigation_generate_messages_cpp)
 #add_dependencies(HSM_NB1_Executive ${catkin_EXPORTED_TARGETS})
 #add_dependencies(HSM_NB1_Executive HSM_Monitor_class hsm_generate_messages_cpp)
 #add_dependencies(HSM_HB_Executive HSM_Heartbeat_Monitor_class hsm_generate_messages_cpp)
 #add_dependencies(HSM_Detector hsm_generate_messages_cpp)
 #add_dependencies(HSM_TEST_node_1 hsm_generate_messages_cpp)
 #add_dependencies(Comm_Monitor_class hsm_generate_messages_cpp)
 #add_dependencies(Escape_Monitor_class hsm_generate_messages_cpp)
 #add_dependencies(HSM_Heartbeat hsm_generate_messages_cpp)
 #add_dependencies(HSM_Heartbeat_Monitor_class hsm_generate_messages_cpp)
 #add_dependencies(info_node comm_generate_messages_cpp robot_control_generate_messages_cpp roboteq_interface_generate_messages_cpp navigation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 #target_link_libraries(TEST_DRIVER_HSM_Detector ${catkin_LIBRARIES})
 #target_link_libraries(TEST_DRIVER_Counter ${catkin_LIBRARIES})
 #target_link_libraries(TEST_DRIVER_Timer ${catkin_LIBRARIES})
 target_link_libraries(HSM_Master_Executive ${catkin_LIBRARIES}
 											Comm_Monitor_class
 											Escape_Monitor_class
 											Counter
 #											Timer
 #											HSM_Heartbeat_Monitor_class
 											robot_pose_monitor
 											)
 
 target_link_libraries(user_input_node ${catkin_LIBRARIES})
 
 #target_link_libraries(info_node ${catkin_LIBRARIES})
 
 #target_link_libraries(HSM_Heartbeat HSM_Detector Timer Counter)
 #target_link_libraries(HSM_NB1_Executive ${catkin_LIBRARIES})
 #target_link_libraries(HSM_HB_Executive ${catkin_LIBRARIES})

 #target_link_libraries(HSM_TEST_node_1 ${catkin_LIBRARIES})
 #target_link_libraries(HSM_TEST_node_2 ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS hsm hsm_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hsm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
