cmake_minimum_required(VERSION 3.0.2)
project(tiago_obstacle_finder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib
  tf
)


################################################
## Declare ROS messages, services and actions ##
################################################


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
add_action_files(
  DIRECTORY
  action
    FILES
    FindObstacles.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package( 
  CATKIN_DEPENDS 
  message_runtime
  actionlib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR} 
)

## Declare a C++ library
add_library(ObjectDetection
  src/ObjectDetection.cpp
  include/${PROJECT_NAME}/ObjectDetection.h
)



# ACTION CLIENT AND SERVER

add_executable(obs_finder_server src/obs_finder_server.cpp)
target_link_libraries(obs_finder_server ${catkin_LIBRARIES} ObjectDetection)
add_dependencies(obs_finder_server ${tiago_obstacle_finder_EXPORTED_TARGETS})

add_executable(obs_finder_client src/obs_finder_client.cpp)
target_link_libraries(obs_finder_client ${catkin_LIBRARIES} ObjectDetection)
add_dependencies(obs_finder_client ${tiago_obstacle_finder_EXPORTED_TARGETS})



