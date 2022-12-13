/**
 * @file TagLocalization_lib.h
 * @author group 01
 * @brief Core of the tag_localization node, performing all thetag detection functionalities
 * 
 * @date 2021-12-28
 * 
 */


#ifndef TAG_LOCALIZATION_MY_LIBRARY_H
#define TAG_LOCALIZATION_MY_LIBRARY_H

// c++ packages
#include <sstream>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

// Tag localization requests
#include <object_manipulation/TagLocaliz.h>




/******************************************/
/***               TYPEDEF              ***/
/******************************************/


/**
 * @brief April tag data structure
 * 
 */
typedef struct {
  geometry_msgs::PoseWithCovarianceStamped pose;
  std::vector<int> id;
} myTAG;




/******************************************/
/***        TagLocalization CLASS       ***/
/******************************************/

/**
 * @brief Class that implements TagLocalization Service SERVER
 * 
 */
class TagLocalization{

public:
  TagLocalization(ros::NodeHandle node, std::string name){

    // the node becomes a service advertiser to return apriltag poses
    tag_server = node.advertiseService(name, &TagLocalization::tagRequestCB, this);
    // subscribe to tag_detections topic
    tag_sub = node.subscribe("/tag_detections",1000, &TagLocalization::tagReceivedCB, this);
  }

  ~TagLocalization() {}

private:
    
  /**
   * @brief Callback to activate when tag_localiz_srv is requested
   * 
   * @param req 
   * @param res 
   * @return true 
   * @return false 
  */
  bool tagRequestCB(object_manipulation::TagLocaliz::Request &req, object_manipulation::TagLocaliz::Response &res);

  /**
   * @brief Callback to activate with apriltag messages
   * 
   * @param msg 
   */
  void tagReceivedCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);


  //// Istances for tag detection
  ros::ServiceServer tag_server;
  ros::Subscriber tag_sub;

  //// ID of the target april tag
  int tag_to_detect;

  //// Pose of the detected april tag
  geometry_msgs::PoseWithCovarianceStamped tag_to_detect_pose;

  //// Is 0 if no target pose detected, 1 otherwise
  bool is_tag_detected = false;  
  
  //// Vector of founded tags
  std::vector< myTAG > found_tags;

};




/******************************************/
/***          TagRequests CLASS         ***/
/******************************************/

/**
 * @brief Class used to implement TagRequests (Service client, basically)
 * 
 */
class TagRequests {

public:

  TagRequests(ros::NodeHandle node){
    tag_client = node.serviceClient<object_manipulation::TagLocaliz>("/tag_localiz_srv");
  }

  /**
   * @brief Send to tag localizer node a service request to read tags
   * 
   */
  void sendRequest();

  /**
   * @brief Add tag to detected_tags vector
   * 
   * @param newTAG 
   */
  void addTag(myTAG newTAG);

  /**
   * @brief Return detected tags
   * 
   * @return std::vector< myTAG > 
   */
  std::vector< myTAG > tagsRead();
  
  /**
   * @brief Print out all the information about the detected tags
   * 
   */
  void displayTags(std::vector< myTAG > tags);

  /**
   * @brief Perform both sendRequest() and tagsRead() in one call
   * 
   * @return std::vector< myTAG > 
   */
  std::vector< myTAG > getTags();


private:

  //// Istance of the detection service
  ros::ServiceClient tag_client;

  //// Vector of all detected tags
  std::vector< myTAG > detected_tags;

};

#endif
