cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  messages
  roscpp
  rospy
  grid_map_ros
  grid_map_msgs
  geometry_msgs
  sensor_msgs
  hsm
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
   Keys_Pressed.msg
   Waypoint.msg
   ROI.msg
   CurrentROI.msg
   DriveSpeeds.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   RegionsOfInterest.srv
   IntermediateWaypoints.srv
   ModifyROI.srv
   SearchMap.srv
   RandomSearchWaypoints.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
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
#  LIBRARIES robot_control
  CATKIN_DEPENDS message_generation message_runtime messages roscpp rospy
#  DEPENDS system_lib
)

add_definitions("-std=gnu++11")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
 add_library(exec src/exec.cpp)
 add_library(action src/action.cpp)
 add_library(idle src/idle.cpp)
 add_library(halt src/halt.cpp)
 add_library(drive_global src/drive_global.cpp)
 add_library(drive_relative src/drive_relative.cpp)
 add_library(grab src/grab.cpp)
 add_library(drop src/drop.cpp)
 add_library(open src/open.cpp)
 add_library(search src/search.cpp)
 add_library(wait src/wait.cpp)
 
 add_library(drive_straight src/drive_straight.cpp)
 add_library(drive_straight_cl src/drive_straight_cl.cpp)
 add_library(drive_pivot src/drive_pivot.cpp)
 add_library(drive_halt src/drive_halt.cpp)
 add_library(grabber_set_slides src/grabber_set_slides.cpp)
 add_library(grabber_set_drop src/grabber_set_drop.cpp)
 add_library(grabber_halt src/grabber_halt.cpp)
 add_library(grabber_idle src/grabber_idle.cpp)
 add_library(vision_halt src/vision_halt.cpp)
 add_library(vision_search src/vision_search.cpp)

 add_library(map_manager src/map_manager.cpp)
 
 add_library(mission_planning src/mission_planning.cpp)
 add_library(pause src/pause.cpp)
 add_library(emergency_escape src/emergency_escape.cpp)
 add_library(avoid src/avoid.cpp)
 add_library(bias_removal src/bias_removal.cpp)
 add_library(next_best_region src/next_best_region.cpp)
 add_library(search_region src/search_region.cpp)
 add_library(examine src/examine.cpp)
 add_library(approach src/approach.cpp)
 add_library(collect src/collect.cpp)
 add_library(confirm_collect src/confirm_collect.cpp)
 add_library(go_home src/go_home.cpp)
 add_library(square_update src/square_update.cpp)
 add_library(deposit_approach src/deposit_approach.cpp)
 add_library(deposit_sample src/deposit_sample.cpp)
 add_library(safe_mode src/safe_mode.cpp)
 
 add_library(safe_pathing src/safe_pathing.cpp)

## Declare a cpp executable
 add_executable(keyboard_control_node src/keyboard_control_node.cpp)
 add_executable(exec_node src/exec_node.cpp)
 add_executable(map_manager_node src/map_manager_node.cpp)
 add_executable(mission_planning_node src/mission_planning_node.cpp)
 add_executable(safe_pathing_node src/safe_pathing_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(keyboard_control_node robot_control_generate_messages_cpp)
 add_dependencies(exec_node robot_control_generate_messages_cpp)
 add_dependencies(map_manager_node robot_control_generate_messages_cpp)
 add_dependencies(mission_planning_node robot_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
 add_dependencies(safe_pathing_node robot_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(keyboard_control_node
   ${catkin_LIBRARIES}
   libncurses.so
 )
 
 target_link_libraries(drive_global action)
 
 target_link_libraries(exec_node
   ${catkin_LIBRARIES}
   exec
   action
   idle
   halt
   drive_global
   drive_relative
   grab
   drop
   open
   search
   wait
   drive_straight
   drive_straight_cl
   drive_pivot
   drive_halt
   grabber_set_slides
   grabber_set_drop
   grabber_halt
   grabber_idle
   vision_halt
   vision_search
 )
 
 target_link_libraries(map_manager_node
   ${catkin_LIBRARIES}
   map_manager
   ${grid_map_ros_LIBRARIES}
 )
 
 target_link_libraries(mission_planning_node
   ${catkin_LIBRARIES}
   mission_planning
   pause
   emergency_escape
   avoid
   bias_removal
   next_best_region
   search_region
   examine
   approach
   collect
   confirm_collect
   go_home
   square_update
   deposit_approach
   deposit_sample
   safe_mode
   voice
 )
 
 target_link_libraries(safe_pathing_node
   ${catkin_LIBRARIES}
   safe_pathing
 )

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
# install(TARGETS robot_control robot_control_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
