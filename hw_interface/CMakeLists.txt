cmake_minimum_required(VERSION 2.8.3)
project(hw_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  messages
  roscpp
  rospy
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
  LIBRARIES Serial_Port_class drive_packet grabber_packet
  CATKIN_DEPENDS message_generation message_runtime messages roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/hw_interface
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(Comm_class	src/Comm_class.cpp)
 add_library(Serial_Port_class src/Serial_Port_class.cpp)
 add_library(UDP_Sender_Port_class src/UDP_Sender_Port_class.cpp)
 add_library(UDP_Sender_Port_class_fault_test src/UDP_Sender_Port_class_fault_test.cpp)
 add_library(UDP_Receiver_Port_class src/UDP_Receiver_Port_class.cpp)
 add_library(nb1_to_i7_packet src/nb1_to_i7_packet.cpp)
 add_library(nb1_to_i7_serial_packet src/nb1_to_i7_serial_packet.cpp)
 add_library(i7_to_nb1_packet src/i7_to_nb1_packet.cpp)
 add_library(nb2_3_to_i7_packet src/nb2_3_to_i7_packet.cpp)
 add_library(drive_packet src/drive_packet.cpp)
 add_library(grabber_packet src/grabber_packet.cpp)
 
 add_library(RoboteqDevice src/RoboteqDevice.cpp)
 add_library(roboteq_class src/roboteq_class.cpp)
 
# add_library(Serial_HSM_Act_class src/Serial_HSM_Act_class.cpp)
# add_library(UDP_HSM_Act_class src/UDP_HSM_Act_class.cpp) 

## Declare a cpp executable
 add_executable(Comm_Node src/communications_node.cpp)
 
 add_executable(speed_node src/speed_node.cpp)
 add_executable(grabber_node src/grabber_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(Comm_Node hw_interface_generate_messages_cpp)
# add_dependencies(Comm_Node hsm_generate_messages_cpp)
 add_dependencies(Comm_Node ${catkin_EXPORTED_TARGETS})
 add_dependencies(nb1_to_i7_packet ${catkin_EXPORTED_TARGETS})
 add_dependencies(nb1_to_i7_packet messages_generate_messages_cpp)
 add_dependencies(nb1_to_i7_serial_packet ${catkin_EXPORTED_TARGETS})
 add_dependencies(nb1_to_i7_serial_packet messages_generate_messages_cpp)
 add_dependencies(i7_to_nb1_packet ${catkin_EXPORTED_TARGETS})
 add_dependencies(i7_to_nb1_packet messages_generate_messages_cpp)
 add_dependencies(drive_packet ${catkin_EXPORTED_TARGETS})
 add_dependencies(drive_packet messages_generate_messages_cpp)
# add_dependencies(drive_packet roboteq_interface_generate_messages_cpp)
 add_dependencies(grabber_packet ${catkin_EXPORTED_TARGETS})
 add_dependencies(grabber_packet messages_generate_messages_cpp)
# add_dependencies(grabber_packet roboteq_interface_generate_messages_cpp)
 add_dependencies(nb2_3_to_i7_packet ${catkin_EXPORTED_TARGETS})
 add_dependencies(nb2_3_to_i7_packet messages_generate_messages_cpp)
 
 add_dependencies(Serial_Port_class messages_generate_messages_cpp)
# add_dependencies(Serial_Port_class hsm_generate_messages_cpp)

 add_dependencies(UDP_Receiver_Port_class messages_generate_messages_cpp)
# add_dependencies(UDP_Receiver_Port_class hsm_generate_messages_cpp)

 add_dependencies(speed_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(grabber_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(roboteq_class ${catkin_EXPORTED_TARGETS})
 add_dependencies(speed_node messages_generate_messages_cpp)
 add_dependencies(grabber_node messages_generate_messages_cpp)
 add_dependencies(roboteq_class messages_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(Serial_Port_class ${catkin_LIBRARIES})
 target_link_libraries(Comm_Node ${catkin_LIBRARIES}
 							Comm_class
 							Serial_Port_class
 							UDP_Sender_Port_class
 							UDP_Sender_Port_class_fault_test
 							UDP_Receiver_Port_class
 							nb1_to_i7_packet
 							nb1_to_i7_serial_packet
 							nb2_3_to_i7_packet
 							i7_to_nb1_packet
 							#Serial_HSM_Act_class
 							#UDP_HSM_Act_class
							)
							
  target_link_libraries(speed_node
   ${catkin_LIBRARIES}
   roboteq_class
 )
  target_link_libraries(grabber_node
   ${catkin_LIBRARIES}
   roboteq_class
 )
 target_link_libraries(roboteq_class 
   ${catkin_LIBRARIES}
   Serial_Port_class
   drive_packet
   grabber_packet)

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
# install(TARGETS hw_interface hw_interface_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hw_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
