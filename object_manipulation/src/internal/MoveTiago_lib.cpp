#include "../include/object_manipulation/MoveTiago_lib.h"


/***************************************/
/***     function implementation     ***/
/***************************************/

// CONSTRUCTOR IN HEADER FILE


bool MoveTiagoClient::navigatePreliminary() 
{

  ROS_INFO("Sending TIAGo to preliminary position...");

  ac.sendGoal(prelim_goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Preliminary position achieved");
      return true;
  }
  else{
      ROS_INFO("Preliminary position has not been achieved");
      ros::shutdown();
      return false;
  }
}


bool MoveTiagoClient::navigateToTable(int target) 
{

  ROS_INFO("Sending TIAGo to target location...");

  if (target == 1) ac.sendGoal(blue_goal);
  else if (target == 2) ac.sendGoal(green_goal);
  else if (target == 3) ac.sendGoal(red_goal);
  else 
  {
    ROS_ERROR("Wrong target position");
    ros::shutdown();
    return false;
  }
  
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Target position achieved");
      return true;
  }
  else{
      ROS_INFO("Target position has not been achieved");
      ros::shutdown();
      return false;
  }
}


object_manipulation::MoveTiagoGoal MoveTiagoClient::returnWantedToTablePose(int target){

    if (target == 1) return blue_goal;
    else if (target == 2) return green_goal;
    else if (target == 3) return red_goal;

}


bool MoveTiagoClient::moveHead(float x_cam, float y_cam, float z_cam)
{
  this->pointHeadClient.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 5;
  // Wait for head controller action server to come up
  while( !(this->pointHeadClient)->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }
 
  if ( iterations == max_iterations )
  throw std::runtime_error("Error in moveHeadClient: head controller action server not available");

  geometry_msgs::PointStamped pointStamped;

  //build the action goal
  control_msgs::PointHeadGoal goal;

  pointStamped.header.frame_id = cameraFrame;

  pointStamped.point.x = x_cam;
  pointStamped.point.y = y_cam;
  pointStamped.point.z = z_cam;

  //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  goal.pointing_frame = cameraFrame;
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(0.1);
  goal.max_velocity = 2;
  goal.target = pointStamped;

  (this->pointHeadClient)-> sendGoal(goal);
  (this->pointHeadClient)-> waitForResult();

  return true;

}


std::vector<std::pair<double, double>> MoveTiagoClient::navigateToDetection() 
{

  ROS_INFO("Sending TIAGo to detection position...");
  std::vector<std::pair<double, double>> result_pos;

  ac.sendGoal(detection_goal); 
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    object_manipulation::MoveTiagoResultConstPtr  result = ac.getResult();
    
     
    if(result->obs_x.empty()) 
    {
      ROS_INFO("Can't detect any table");
      return result_pos;
    }

    for (int i = 0; i < result->obs_x.size(); i++)
    {
      std::pair<double, double> pos;
      pos.first = result->obs_x[i];
      pos.second = result->obs_y[i];
      std::cout <<"[DETECTION] detected obstacle #"<< i << " x: " << pos.first << " y: " << pos.second << std::endl;
      result_pos.push_back(pos);
    }
    
    ROS_INFO("Detection complete");
    return result_pos;
  }
  else{
    ROS_INFO("Starting position has not been achieved");
    ros::shutdown();
    return result_pos;
  }
}


object_manipulation::MoveTiagoGoal MoveTiagoClient::returnDetectionGoal(){

    return detection_goal;
}


bool MoveTiagoClient::navigate(double x, double y, double theta) 
{

  ROS_INFO("Sending TIAGo to position...");

  object_manipulation::MoveTiagoGoal goal;
  goal.pos_x = x;
  goal.pos_y = y;
  goal.angle_theta = theta;

  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Position achieved");
      return true;
  }
  else{
      ROS_INFO("Position has not been achieved");
      ros::shutdown();
      return false;
  }
}
