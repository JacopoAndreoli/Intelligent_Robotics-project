/**
 * @file tag_localization.cpp
 * @brief Implementation of an action client that moves the head of TIAGo towards the table, locates
 * apriltag of the object to pick up and gives back its pose wrt base_link
 *
 * @date 2021-12-28
 *
 */

// c++ packages
#include <sstream>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

// Tag localization requests
#include <object_manipulation/TagLocaliz.h>

// OUR LIBRARY FOR TAG LOCALIZATION
#include "../include/object_manipulation/TagLocalization_lib.h"

/***************************************/
/***              MAIN               ***/
/***************************************/


int main (int argc, char **argv) {

  // Init the ROS node
  ros::init(argc, argv, "tag_localization");
  ros::NodeHandle n_tag;

  TagLocalization Localizer(n_tag,"/tag_localiz_srv");

  ROS_INFO_STREAM("Tag localization service started");

  ros::spin();
  return 0;
}
