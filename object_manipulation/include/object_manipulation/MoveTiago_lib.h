/**
 * @file MoveTiago_lib.h
 * @author group 01
 * @brief Class that performs Tiago’s navigation inside the room using the HW1 action server, 
 * as well as perform the little movements of Tiago’s head needed to correctly detect all 
 * the april tag in the table scene
 * 
 * @date 2022-01-02
 * 
 */

#ifndef MOVE_TIAGO_MY_LIBRARY_H
#define MOVE_TIAGO_MY_LIBRARY_H

// c++ packages
#include <sstream>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>

// Action for moving the robot to the table
#include <object_manipulation/MoveTiagoAction.h>
#include <tiago_obstacle_finder/FindObstaclesAction.h>




typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;
static const std::string cameraFrame = "/xtion_rgb_optical_frame";


/**
 * @brief Class used to implement all the motions up to the table + head motions. 
 * It's a client for obstacle_finder_server and head_controller
 * 
 */
class MoveTiagoClient
{

public:

  MoveTiagoClient() : ac("obstacle_finder", true) {

    // Position in which the table can be easily reached
    prelim_goal.pos_x = 8.15;
    prelim_goal.pos_y = -1.3; //-0.9;
    prelim_goal.angle_theta = -1.57;
    
    // Position to pick blue
    blue_goal.pos_x = 8.0;
    blue_goal.pos_y = -2.08;
    blue_goal.angle_theta = -1.58;
  
    // Position to pick green
    green_goal.pos_x = 7.6;
    green_goal.pos_y = -3.9;
    green_goal.angle_theta = +1.57;
    
    // Position to pick red
    red_goal.pos_x = 7.5;
    red_goal.pos_y = -1.9;
    red_goal.angle_theta = -1.58;

    // Position where detect placing tables
    detection_goal.pos_x = 11;
    detection_goal.pos_y = -3.0;
    detection_goal.angle_theta =  +1.57;

  
    ROS_INFO("Waiting for obstacle_finder server to start.");
    ac.waitForServer();
    ROS_INFO("Server started");

  }

  /**
   * @brief Take TIAGo to preliminary position in which the table can be easily reached
   * 
   * @return true 
   * @return false 
   */
  bool navigatePreliminary();

  /**
   * @brief Take TIAGo to the table according to picking object
   * 
   * @param target 
   * @return true 
   * @return false 
   */
  bool navigateToTable(int target);

  /**
   * @brief Return wanted pose of TIAGo to pick an object
   *
   * @param target
   * @return object_manipulation::MoveTiagoGoal  Goal to reach
   */
  object_manipulation::MoveTiagoGoal returnWantedToTablePose(int target);

  /**
   * @brief Take TIAGo to a position where it can detect placing tables, detect the tables position
   * 
   * @return std::vector<std::pair<double, double>>  Position of tables
   */
  std::vector<std::pair<double, double>> navigateToDetection();

  /**
   * @brief Just return detection position (useful when we go back after placing an object)
   *
   * @return object_manipulation::MoveTiagoGoal  detection_goal
   */
  object_manipulation::MoveTiagoGoal returnDetectionGoal();

  /**
   * @brief Take TIAGo to a custom position
   * 
   * @param x 
   * @param y 
   * @param theta 
   * @return true 
   * @return false 
   */
  bool navigate(double x, double y, double theta);

  /**
   * @brief Create a ROS action client to move TIAGo's head
   * 
   * @param x_cam 
   * @param y_cam 
   * @param z_cam 
   * @return true 
   * @return false 
   */ 
  bool moveHead(float x_cam, float y_cam, float z_cam);


private:

  //// Action istances
  PointHeadClientPtr pointHeadClient;
  actionlib::SimpleActionClient<object_manipulation::MoveTiagoAction> ac;

  //// Goal positions
  object_manipulation::MoveTiagoGoal blue_goal, green_goal, red_goal, prelim_goal, detection_goal;

};


#endif
