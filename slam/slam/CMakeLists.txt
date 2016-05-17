cmake_minimum_required(VERSION 2.8.3)
project(slam)

set(libpointmatcher_DIR /usr/local/lib/cmake/pointmatcher)

## Find catkin libraries

find_package(catkin REQUIRED 
	roscpp 
	rosbag 
	std_msgs 
	geometry_msgs 
	nav_msgs
	tf 
	tf2 
	tf_conversions 
	message_filters 
	libpointmatcher_ros 
	message_generation 
	libpointmatcher 
	eigen_conversions
	interactive_markers
	messages
	message_runtime
	pcl_conversions 
  	pcl_ros)

find_package(PkgConfig REQUIRED)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)


catkin_package (
	CATKIN_DEPENDS 
	roscpp rosbag 
	std_msgs 
	geometry_msgs 
	nav_msgs
	tf 
	tf2 
	tf_conversions 
	message_filters 
	libpointmatcher_ros 
 	libpointmatcher 
 	eigen_conversions
 	interactive_markers
	messages
	message_runtime
    message_generation
    pcl_conversions 
  	pcl_ros
	 
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# c++ 0x
#if("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang") 
#	set(CMAKE_COMPILER_IS_CLANGXX)
#endif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang")
if(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		add_definitions(-std=gnu++0x)
	else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
	endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
else(CMAKE_COMPILER_IS_GNUCXX)
	message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)
# well, this breaks libpointmatcher on non-unix platform,
# please contribute the define for your favorite compiler
# It must support (compiler support in parenthesis):
# - N2351: shared pointers in standard
# - N2672: Initializer lists (gcc)
# - N1984: auto-typed variables (gcc, clang)




set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DHAVE_YAML_CPP)

include_directories(include 		   
		${catkin_INCLUDE_DIRS} )

#add_library(Position src/Position.cpp)
#target_link_libraries(Position ${catkin_LIBRARIES})

add_library(Keyframe src/Keyframe.cpp)
target_link_libraries(Keyframe ${catkin_LIBRARIES})

#add_library(keyframe_Position src/keyframe_Position.cpp)
#target_link_libraries(keyframe_Position ${catkin_LIBRARIES})

#add_library(Mapper src/Mapper.cpp)
#target_link_libraries(Mapper ${catkin_LIBRARIES})

add_executable(Keyframe_node src/Keyframe_node.cpp)
add_dependencies(Keyframe_node ${PROJECT_NAME}_gencpp)
target_link_libraries(Keyframe_node ${catkin_LIBRARIES} Keyframe)

#add_executable(Mapper_node src/Mapper_node.cpp)
#add_dependencies(Mapper_node ${PROJECT_NAME}_gencpp)
#target_link_libraries(Mapper_node ${catkin_LIBRARIES} Mapper keyframe_Position)

#add_executable(keymapper src/keymapper.cpp)
#add_dependencies(keymapper ${PROJECT_NAME}_gencpp)
#target_link_libraries(keymapper ${catkin_LIBRARIES})
