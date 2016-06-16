cmake_minimum_required(VERSION 2.8.3)
project(cataglyphis_gui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  grid_map_ros
  grid_map_msgs
  messages
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

set(Qt5Core_DIR /opt/Qt5.5.1/5.5/gcc_64/lib/cmake/Qt5Core)

# Find the QtWidgets library
find_package(Qt5Core)
#find_package(Qt5Widgets)
SET(QT_USE_QTXML 1)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

#-fPIC allows static libraries of code to be linked together
#Qt requirement. Doesn't appear to affect anything ROS.
add_definitions("-std=gnu++11 -fPIC")

#search this directory for libraries 
link_directories(  /opt/Qt5.5.1/5.5/gcc_64/lib)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   messages
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
#  INCLUDE_DIRS include
#  LIBRARIES robot_control
  CATKIN_DEPENDS message_generation message_runtime messages roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  qt_project_src
  qt_project_src/build-cataglyphis_gui 
  qt_project_src/cataglyphis_gui
  /opt/Qt5.5.1/5.5/gcc_64/lib
  /opt/Qt5.5.1/5.5/gcc_64/include
  /opt/Qt5.5.1/5.5/gcc_64/include/Enginio
  /opt/Qt5.5.1/5.5/gcc_64/include/Qt3DCollision
  /opt/Qt5.5.1/5.5/gcc_64/include/Qt3DCore
  /opt/Qt5.5.1/5.5/gcc_64/include/Qt3DInput
  /opt/Qt5.5.1/5.5/gcc_64/include/Qt3DLogic
  /opt/Qt5.5.1/5.5/gcc_64/include/Qt3DQuick
  /opt/Qt5.5.1/5.5/gcc_64/include/Qt3DQuickRenderer
  /opt/Qt5.5.1/5.5/gcc_64/include/Qt3DRenderer
  /opt/Qt5.5.1/5.5/gcc_64/include/QtBluetooth
  /opt/Qt5.5.1/5.5/gcc_64/include/QtCLucene
  /opt/Qt5.5.1/5.5/gcc_64/include/QtConcurrent
  /opt/Qt5.5.1/5.5/gcc_64/include/QtCore
  /opt/Qt5.5.1/5.5/gcc_64/include/QtDBus
  /opt/Qt5.5.1/5.5/gcc_64/include/QtDeclarative
  /opt/Qt5.5.1/5.5/gcc_64/include/QtDesigner
  /opt/Qt5.5.1/5.5/gcc_64/include/QtDesignerComponents
  /opt/Qt5.5.1/5.5/gcc_64/include/QtGui
  /opt/Qt5.5.1/5.5/gcc_64/include/QtHelp
  /opt/Qt5.5.1/5.5/gcc_64/include/QtLocation
  /opt/Qt5.5.1/5.5/gcc_64/include/QtMultimedia
  /opt/Qt5.5.1/5.5/gcc_64/include/QtMultimediaQuick_p
  /opt/Qt5.5.1/5.5/gcc_64/include/QtMultimediaWidgets
  /opt/Qt5.5.1/5.5/gcc_64/include/QtNetwork
  /opt/Qt5.5.1/5.5/gcc_64/include/QtNfc
  /opt/Qt5.5.1/5.5/gcc_64/include/QtOpenGL
  /opt/Qt5.5.1/5.5/gcc_64/include/QtOpenGLExtensions
  /opt/Qt5.5.1/5.5/gcc_64/include/QtPlatformHeaders
  /opt/Qt5.5.1/5.5/gcc_64/include/QtPlatformSupport
  /opt/Qt5.5.1/5.5/gcc_64/include/QtPositioning
  /opt/Qt5.5.1/5.5/gcc_64/include/QtPrintSupport
  /opt/Qt5.5.1/5.5/gcc_64/include/QtQml
  /opt/Qt5.5.1/5.5/gcc_64/include/QtQmlDevTools
  /opt/Qt5.5.1/5.5/gcc_64/include/QtQuick
  /opt/Qt5.5.1/5.5/gcc_64/include/QtQuickParticles
  /opt/Qt5.5.1/5.5/gcc_64/include/QtQuickTest
  /opt/Qt5.5.1/5.5/gcc_64/include/QtQuickWidgets
  /opt/Qt5.5.1/5.5/gcc_64/include/QtScript
  /opt/Qt5.5.1/5.5/gcc_64/include/QtScriptTools
  /opt/Qt5.5.1/5.5/gcc_64/include/QtSensors
  /opt/Qt5.5.1/5.5/gcc_64/include/QtSerialPort
  /opt/Qt5.5.1/5.5/gcc_64/include/QtSql
  /opt/Qt5.5.1/5.5/gcc_64/include/QtSvg
  /opt/Qt5.5.1/5.5/gcc_64/include/QtTest
  /opt/Qt5.5.1/5.5/gcc_64/include/QtUiPlugin
  /opt/Qt5.5.1/5.5/gcc_64/include/QtUiTools
  /opt/Qt5.5.1/5.5/gcc_64/include/QtWebChannel
  /opt/Qt5.5.1/5.5/gcc_64/include/QtWebEngine
  /opt/Qt5.5.1/5.5/gcc_64/include/QtWebEngineWidgets
  /opt/Qt5.5.1/5.5/gcc_64/include/QtWebKit
  /opt/Qt5.5.1/5.5/gcc_64/include/QtWebKitWidgets
  /opt/Qt5.5.1/5.5/gcc_64/include/QtWebSockets
  /opt/Qt5.5.1/5.5/gcc_64/include/QtWebView
  /opt/Qt5.5.1/5.5/gcc_64/include/QtWidgets
  /opt/Qt5.5.1/5.5/gcc_64/include/QtX11Extras
  /opt/Qt5.5.1/5.5/gcc_64/include/QtXml
  /opt/Qt5.5.1/5.5/gcc_64/include/QtXmlPatterns
  /opt/Qt5.5.1/5.5/gcc_64/include/QtZlib

  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(ros_workers                         qt_project_src/cataglyphis_gui/ros_workers.cpp)

add_library(cataglyphis_gui                     qt_project_src/cataglyphis_gui/cataglyphis_gui.cpp)
add_library(cataglyphis_startup_form_main       qt_project_src/cataglyphis_gui/cataglyphis_startup_form_main.cpp)

add_library(map_viewer                          qt_project_src/cataglyphis_gui/map_viewer.cpp)

add_library(init_step_one                       qt_project_src/cataglyphis_gui/init_step_one.cpp)
add_library(bias_removal_form                   qt_project_src/cataglyphis_gui/bias_removal_form.cpp)

add_library(generic_error_dialog_form           qt_project_src/cataglyphis_gui/generic_error_dialog_form.cpp)

#add_library(moc_cataglyphis_gui                 qt_project_src/build-cataglyphis_gui/moc_cataglyphis_gui.cpp)
#add_library(moc_cataglyphis_startup_form_main   qt_project_src/build-cataglyphis_gui/moc_cataglyphis_startup_form_main.cpp)
#add_library(moc_init_step_one                   qt_project_src/build-cataglyphis_gui/moc_init_step_one.cpp)
#add_library(moc_map_viewer                      qt_project_src/build-cataglyphis_gui/moc_map_viewer.cpp)
#add_library(moc_generic_error_dialog            qt_project_src/build-cataglyphis_gui/moc_generic_error_dialog.cpp)
add_library(moc_ros_workers                     qt_project_src/build-cataglyphis_gui/moc_ros_workers.cpp)

add_library(gui_resources                       qt_project_src/build-cataglyphis_gui/qrc_resources.cpp)

add_executable(cataglyphis_gui_node             src/cataglyphis_gui_node.cpp)

target_link_libraries(cataglyphis_gui_node
    ${catkin_LIBRARIES}
    
    libQt5Widgets.so
    gui_resources

#    moc_cataglyphis_gui
#    moc_cataglyphis_startup_form_main
#    moc_init_step_one
#    moc_generic_error_dialog
#    moc_map_viewer

    cataglyphis_gui
    cataglyphis_startup_form_main
    generic_error_dialog_form
    init_step_one
    bias_removal_form
    map_viewer
    moc_ros_workers
    ros_workers
    )
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(cataglyphis_gui
#   src/${PROJECT_NAME}/cataglyphis_gui.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(cataglyphis_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(cataglyphis_gui_node src/cataglyphis_gui_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(cataglyphis_gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(cataglyphis_gui_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS cataglyphis_gui cataglyphis_gui_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cataglyphis_gui.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
