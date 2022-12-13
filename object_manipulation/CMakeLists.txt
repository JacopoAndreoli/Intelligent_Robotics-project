cmake_minimum_required(VERSION 3.0.2)
project(object_manipulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  sensor_msgs
  control_msgs
  geometry_msgs
  tf
  apriltag_ros
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  gazebo_ros_link_attacher
  tiago_obstacle_finder
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
 add_service_files(
   FILES
   Objs.srv
   TagLocaliz.srv
   arm_handler.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   MoveTiago.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   geometry_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package( 
  CATKIN_DEPENDS 
  message_runtime
  actionlib
  moveit_ros_planning_interface
  moveit_visual_tools
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR} 
)

## Declare a C++ library

add_library(TagLocalization_lib
  src/internal/TagLocalization_lib.cpp
  include/${PROJECT_NAME}/TagLocalization_lib.h
)

add_library(MoveTiago_lib
  src/internal/MoveTiago_lib.cpp
  include/${PROJECT_NAME}/MoveTiago_lib.h
)

add_library(Gripper_lib
  src/internal/GripperManager_lib.cpp
  src/internal/GripperAttacher_lib.cpp
  include/${PROJECT_NAME}/GripperManager_lib.h
  include/${PROJECT_NAME}/GripperAttacher_lib.h
)

add_library(CollisionObjects_lib
  src/internal/CollisionObjects_lib.cpp
  include/${PROJECT_NAME}/CollisionObjects_lib.h
)

add_library(ArmHandler_lib
  src/internal/ArmHandler_lib.cpp
  include/${PROJECT_NAME}/ArmHandler_lib.h
)


add_executable(pandp_manager src/pandp_manager.cpp)
target_link_libraries(pandp_manager ${catkin_LIBRARIES} TagLocalization_lib MoveTiago_lib Gripper_lib)
#add_dependencies(pandp_manager ${object_manipulation_EXPORTED_TARGETS})

add_executable(tag_localization src/tag_localization.cpp)
target_link_libraries(tag_localization ${catkin_LIBRARIES} TagLocalization_lib)
#add_dependencies(tag_localization ${object_manipulation_EXPORTED_TARGETS})

add_executable(arm_handler src/arm_handler.cpp)
target_link_libraries(arm_handler ${catkin_LIBRARIES} ArmHandler_lib CollisionObjects_lib Gripper_lib)
#add_dependencies(pandp_manager ${object_manipulation_EXPORTED_TARGETS})





