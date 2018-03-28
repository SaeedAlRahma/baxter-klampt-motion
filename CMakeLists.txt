cmake_minimum_required(VERSION 2.8.3)
project(baxter_saeed)

SET(KLAMPT_ROOT /home/dukehal/Klampt)
SET(CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

FIND_PACKAGE(Klampt REQUIRED)
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})

################################################
## Declare ROS messages, services and actions ##
################################################

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
#  LIBRARIES baxter_saeed
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(
  ${KLAMPT_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(baxter_saeed
#   src/${PROJECT_NAME}/baxter_saeed.cpp
# )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(baxter_saeed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(baxter_saeed_node src/baxter_saeed_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(baxter_saeed_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(baxter_saeed_node
#   ${catkin_LIBRARIES}
# )

