cmake_minimum_required(VERSION 2.8.3)
project(trajectory_gps_imu)
add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
		roscpp	
		geometry_msgs
		message_generation
		nav_msgs
		roscpp
		sensor_msgs
		std_msgs
		message_filters
)

find_package(GTSAM REQUIRED) 
find_package(GTSAMCMakeTools)
#include(GtsamBuildTypes) # Load build type flags and default to Debug mode
#include(GtsamTesting)    # Easy functions for creating unit tests and scripts
#include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation
find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS
	thread
	system
)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp
	DEPENDS OpenCV 
		geometry_msgs
		message_runtime
		nav_msgs
		roscpp
		sensor_msgs
		std_msgs
		message_filters
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${GTSAM_INCLUDE_DIRS}
"/usr/include/eigen3"
)
#add_library(${PROJECT_NAME} lib/lib.cpp)
add_executable(trajectory main.cpp lib/lib.cpp)
#add_subdirectory(lib)
target_link_libraries(trajectory 
			${catkin_LIBRARIES} 
			${GTSAM_INCLUDE_DIRS}
			${OpenCV_INCLUDE_DIRS}
			gtsam
)


