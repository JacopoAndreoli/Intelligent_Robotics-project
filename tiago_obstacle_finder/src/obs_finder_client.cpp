/**
 * @file obs_finder_client.cpp
 * @brief Implementation of an action client that receives the input position from the user,
 * send it to an action server, and receives back the location of detected obstacles in that position.
 * 
 * @date 2021-12-19
 *
 * 
 */


// c++ packages
#include <sstream>
#include <string>

// ros packages
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tiago_obstacle_finder/FindObstaclesAction.h>

// Action server
#include <include/tiago_obstacle_finder/ObjectDetection.h>


// Convert the tasks state received from the server into strings
std::string fromEnumToFeedback(int state){

  std::string state_string;
  switch(state){
    case 0:
      state_string = "NONE";
      break;

    case 1:
      state_string = "NAVIGATION";
      break;

    case 2:
      state_string = "FAIL";
      break;

    case 3:
      state_string = "IN_POSITION";
      break;

    case 4:
      state_string = "DETECTION_COMPLETE";
    break;

  }

  return state_string;
}



// Class that implement the action client
class FindObstaclesClient
{

public:

  FindObstaclesClient(std::string p_x, std::string p_y, std::string theta) : ac("obstacle_finder", true)
  {
    // Convert user input strings to goal values
    goal.pos_x = std::atof(p_x.c_str());
    goal.pos_y = std::atof(p_y.c_str());
    goal.angle_theta = std::atof(theta.c_str());
    
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal..");
  }

  // Send goal
  void start()
  {
    ac.sendGoal(goal,
                boost::bind(&FindObstaclesClient::doneCb, this, _1, _2),
                boost::bind(&FindObstaclesClient::activeCb, this),
                boost::bind(&FindObstaclesClient::feedbackCb, this, _1)); 
  }

  // Show the results
  void doneCb(const actionlib::SimpleClientGoalState& state, const  tiago_obstacle_finder::FindObstaclesResultConstPtr& result)
  {
    if(result->obs_x.empty()){
      ROS_INFO("Finished in state [%s]", state.toString().c_str());
      ROS_INFO("robot not able to reach the target goal; obstacle detection does not start");
      }
    else{
      ROS_INFO("Finished in state [%s]", state.toString().c_str());
      
      for ( int i = 0; i < result->obs_x.size(); i++ ){
        ROS_INFO("position of obstacle #%i: (%f , %f)", i+1, result->obs_x[i], result->obs_y[i]);
      }
    
    }
  
    ros::shutdown();
  }

  void activeCb()
  {
    ROS_INFO("Goal sent!");
  }

  // Show tasks state when receives feedback from server
  void feedbackCb(const  tiago_obstacle_finder::FindObstaclesFeedbackConstPtr& feedback)
  {
    std::string state_string = fromEnumToFeedback(feedback->state.back());
    ROS_INFO("Got Feedback - task state: [%s]", state_string.c_str());
  }

  private:
    actionlib::SimpleActionClient<tiago_obstacle_finder::FindObstaclesAction> ac;
    tiago_obstacle_finder::FindObstaclesGoal goal;
};


int main (int argc, char **argv) {

  if (argc < 4) {
    std::cerr << "Not enought arguments for: " << argv[0] 
            << std::endl 
            << "Please enter a coordinates pair x y for the target position [m,m] and orientation [rad]."
            << std::endl
            << "Note that the coordinates are referred to the map reference frame, that coincides with robot's initial position frame"
            << std::endl
            << "Example: 2.5 3.65  1"
            << std::endl; 
    return 1;
  }

  // Start the action client
  ros::init(argc, argv, "obstacle_finder_client");
  FindObstaclesClient findObstaclesClient( argv[1], argv[2],argv[3]);
  findObstaclesClient.start();
  ros::spin(); 
  
  return 0;
}


