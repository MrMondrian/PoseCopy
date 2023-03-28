cmake_minimum_required(VERSION 3.10)
project(controls)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
add_definitions(-Wall -Werror)


###################################
## catkin find required packages ##
###################################

find_package(PkgConfig)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    roslint
    rospy
    urdf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(

  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    rospy
    urdf
  DEPENDS
)


###################################
## catkin install commands ########
###################################

catkin_install_python(PROGRAMS scripts/controls.py scripts/pose_estimation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
	include
  	${catkin_INCLUDE_DIRS}
)


