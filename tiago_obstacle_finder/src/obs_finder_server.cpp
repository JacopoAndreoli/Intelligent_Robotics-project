/**
 * @file obs_finder_server.cpp
 * @brief Implementation of an action server that receives as input a position to be
 * reached (position and orientation). In that position performs an obstacle detection using 
 * the laser scan informations.
 * The server provides as result, the positions of the obstacles detected, and as feedback
 * the current task that it is performing.
 * The obstacles are supposed to be cylindrical.
 * 
 * @date 2021-12-19
 * 
 * 
 */


// c++ packages
#include <math.h>
#include <vector>

// ros packages
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_obstacle_finder/FindObstaclesAction.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

// our class for the obstacles detection
#include "include/tiago_obstacle_finder/ObjectDetection.h"



/**************************************/
/***         fcn definition         ***/
/**************************************/

/* Function used to novigate the robot to the target position unsing move_base */
bool navigateTo(float x, float y, float theta);


/* Class that implement the server and all the actions needed to navigate in a target position and */
/* detect the obstacles in that given position */
class FindObstaclesAction{
    
    private:

    enum TaskState { 
        NONE,
        NAVIGATION, 
        FAIL,
        IN_POSITION,
        DETECTION_COMPLETE
    };

    public:
    
    FindObstaclesAction(ros::NodeHandle node, std::string name, ObjectDetection* detector_) 
                : as_(node, name, boost::bind(&FindObstaclesAction::executeCB, this, _1), false), action_name_(name){

        as_.start();
        detector = detector_;
    }

    ~FindObstaclesAction(void){}

    // callback function for perform all the action needed when the client sends a request 
    // this is the core of the server functioning
    void executeCB(const tiago_obstacle_finder::FindObstaclesGoalConstPtr &goal);

    // callback function to retrieve /move_base/feedback
    void mbfeedbackCB(const move_base_msgs::MoveBaseActionFeedbackConstPtr &mb_fb);


    protected:

    //// Action server details
    actionlib::SimpleActionServer<tiago_obstacle_finder::FindObstaclesAction> as_;
    std::string action_name_;
    tiago_obstacle_finder::FindObstaclesFeedback feedback_;
    tiago_obstacle_finder::FindObstaclesResult result_;

    //// Istance of detection class
    ObjectDetection* detector;

    //// Pose of the robot
    geometry_msgs::Pose feedback_pose;

    //// Current task 
    TaskState current_state = NONE;
};




/***************************************/
/***              MAIN               ***/
/***************************************/

int main(int argc, char** argv){

    // initialize the node
    ros::init(argc, argv, "obstacle_finder");
    ros::NodeHandle nh_;

    // create an ObjectDetection class instance
    ObjectDetection detector;

    // create the server
    FindObstaclesAction obs_finder(nh_,"obstacle_finder", &detector);

    // subscribe to /scan topic, and activate object_detection::LaserCallback callback fcn
    ros::Subscriber sub_ = nh_.subscribe("/scan", 1, &ObjectDetection::LaserCallback, &detector);
    ros::Subscriber sub_mb_fb = nh_.subscribe("/move_base/feedback", 1, &FindObstaclesAction::mbfeedbackCB, &obs_finder);

    ros::spin();
    return 0;
}




/********************************************/
/***          fcn implementation          ***/
/********************************************/



bool navigateTo(float x, float y, float theta) {

    // calculating the corresponding position and orientation quaternion
    float requested_pos_x = x;
    float requested_pos_y = y;
    float requested_pos_z = 0.0;

    float requested_orient_w = cos((theta)/2);
    float requested_orient_x = 0.0;
    float requested_orient_y = 0.0;
    float requested_orient_z = sin((theta)/2);

    // sending the new position to /move_base server
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_mb("move_base", true);
    ROS_INFO("Waiting for /move_base to start.");
    ac_mb.waitForServer(); //will wait for infinite time
    ROS_INFO("/move_base started, sending goal.");

    //we'll send a goal to the robot to move to the precomputed position (wrt map)
    move_base_msgs::MoveBaseGoal pos_goal;
    pos_goal.target_pose.header.frame_id = "map";
    pos_goal.target_pose.pose.position.x = requested_pos_x;
    pos_goal.target_pose.pose.position.y = requested_pos_y;
    pos_goal.target_pose.pose.position.z = requested_pos_z;
    pos_goal.target_pose.pose.orientation.w = requested_orient_w;
    pos_goal.target_pose.pose.orientation.z = requested_orient_z;
    pos_goal.target_pose.pose.orientation.x = requested_orient_x;
    pos_goal.target_pose.pose.orientation.y = requested_orient_y;

    ROS_INFO("Sending goal to /move_base");
    ac_mb.sendGoal(pos_goal);

    bool finished_before_timeout = ac_mb.waitForResult();

    if(ac_mb.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Target position achieved");
        return true;
    }
    else{
        ROS_INFO("Target position has not been achieved");
        return false;
    }

}



void FindObstaclesAction::executeCB(const tiago_obstacle_finder::FindObstaclesGoalConstPtr &goal){

    ros::Rate r(100);
    r.sleep();

    feedback_.state.clear();
    result_.obs_x.clear();
    result_.obs_y.clear();

    /* NAVIGATION TO THE TARGET */

    // publish the new state: NAVIGATION
    current_state = NAVIGATION;
    feedback_.state.push_back(current_state);
    as_.publishFeedback(feedback_);

    bool robot_on_target = navigateTo(goal->pos_x, goal->pos_y, goal->angle_theta);

    if(robot_on_target) current_state = IN_POSITION;
    else current_state = FAIL;

    // publish the new state
    feedback_.state.push_back(current_state);
    as_.publishFeedback(feedback_);

    /* FAIL TO REACH THE TARGET POSITION */

    if (current_state == FAIL){
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
    }

    /* ROBOT IN POSITION: START OBSTACLES DETECTION */

    if (current_state == IN_POSITION){

        // Scan measurement request 
        detector->scanRequest();
        ROS_INFO("Waiting for measurements to be ready...");
        while(!detector->scanReceived()){;}

        ROS_INFO("... Measurements received!");

        // Just for debug
        //detector->showMeas();

        // Find the obstacles centers wrt robot frame
        std::vector <Point> centers_robot_frame = detector->FitCircle();

        // Convert from robot frame to map frame
        std::vector <Point> centers_map_frame;

        // Retrieve transformation from laser frame to base frame, and then to map frame
        tf::TransformListener TFlistener;
        tf::StampedTransform transform;
        ROS_INFO("Waiting for frames transformation...");
        TFlistener.waitForTransform("/base_link", "/base_laser_link", ros::Time::now(), ros::Duration(4.0));
        TFlistener.lookupTransform("/base_link", "/base_laser_link", ros::Time(0), transform);
        ROS_INFO("... Found frame transformation");

        // Retrieve obsracles centers wrt map frame
        for (int idx = 0; idx < centers_robot_frame.size(); idx ++){
            Point center_rf = centers_robot_frame[idx];
            Point center_mf;

            float distance_robot_center = sqrt((center_rf.x + transform.getOrigin().x())*(center_rf.x + transform.getOrigin().x()) + (center_rf.y + transform.getOrigin().y())*(center_rf.y + transform.getOrigin().y()));
            float angle_robot_center = transform.getRotation().z() + atan2(center_rf.y,center_rf.x);

            // Angle between map and base_link
            float map_base_angle = 2*atan2(feedback_pose.orientation.z, feedback_pose.orientation.w);

            center_mf.x = feedback_pose.position.x + distance_robot_center*cos(map_base_angle + angle_robot_center);
            center_mf.y = feedback_pose.position.y + distance_robot_center*sin(map_base_angle + angle_robot_center);
            centers_map_frame.push_back(center_mf);
        }

        // Store the obstacle positions
        for(int i = 0; i < centers_map_frame.size(); i++){
            result_.obs_x.push_back(centers_map_frame[i].x);
            result_.obs_y.push_back(centers_map_frame[i].y);
        }

        // Send the results
        current_state = DETECTION_COMPLETE;
        feedback_.state.push_back(current_state);

        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
        r.sleep();
    }

}

// Callback function to retrieve robot pose from /move_base/feedback
void FindObstaclesAction::mbfeedbackCB(const move_base_msgs::MoveBaseActionFeedbackConstPtr &mb_fb){

    feedback_pose.position.x = mb_fb->feedback.base_position.pose.position.x;
    feedback_pose.position.y = mb_fb->feedback.base_position.pose.position.y;
    feedback_pose.orientation.w = mb_fb->feedback.base_position.pose.orientation.w;
    feedback_pose.orientation.z = mb_fb->feedback.base_position.pose.orientation.z;
}
