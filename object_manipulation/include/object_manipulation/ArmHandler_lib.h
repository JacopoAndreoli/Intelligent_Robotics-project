/**
 * @file ArmHandler_lib.h
 * @author group 01
 * @brief Class that provides the services and functions used by arm_handler node. 
 * In particular, this class provides executing instruction for the main node, poses for the Tiago’s arm, 
 * and manage the use ofcollision objects and the gripper through the following remaining classes
 * 
 * @date 2022-01-21
 * 
 * 
 */

// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <object_manipulation/arm_handler.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <typeinfo>
#include <sstream>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>


// Our librearies implementation
#include "GripperManager_lib.h"
#include "GripperAttacher_lib.h"
#include "CollisionObjects_lib.h"



/**
 * @brief Class that provides the services and functions used by arm_handler node. 
 * In particular, this class provides executing instruction for the main node, poses for the Tiago’s arm, 
 * and manage the use ofcollision objects and the gripper through the following remaining classes.
 * State (flag) defition:
 * state  0: the service is ready, and Tiago’s arm is in home or inert position 
 * state  1: the picking service has been called and Tiago is going to grasp the object 
 * state  2: Tiago has grabbed the object and has its arm in a safe position to travel 
 * state  3: the placing service has been called and Tiago is going to place the object 
 * state  4: Tiago has placed the object and has its arm in a safe position to travel 
 * state -1: Tiago encountered problems during the motion
 * 
 */
class ArmHandler
{

public:

    ArmHandler() {};

    ~ArmHandler() {};


    /**
     * @brief Callback for the arm hndler Pick service
     * 
     * @param req Tags (objects) identifyed by ID and pose, target object's ID
     * @param res 
     * @return true 
     * @return false 
     */
    bool handlingPICKRequestCB(object_manipulation::arm_handler::Request &req, object_manipulation::arm_handler::Response &res);

    /**
     * @brief Callback for the arm hndler Place service
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool handlingPLACERequestCB(object_manipulation::arm_handler::Request &req, object_manipulation::arm_handler::Response &res);

    /**
     * @brief Specify to the main function which manimulation action start with Moveit!
     * 
     * @return true 
     * @return false 
     */
    inline int execute() { return flag; }

    /**
     * @brief reset flag
     */
    inline void resetFlag() { flag = 0; }

    /**
     * @brief set custom flag's value
     */
    inline void setFlag(int v) { flag = v; }

    /**
     * @brief Get the Tags object
     * 
     * @return std::vector<myTAG> 
     */
    inline std::vector<myTAG> getTags() { return detected_pose; }

    /**
     * @brief Get the Target I D object
     * 
     * @return int 
     */
    inline int getTargetID() { return target_id; }

    /**
     * @brief Get the Goal Pose object
     * 
     * @return geometry_msgs::PoseStamped 
     */
    inline geometry_msgs::PoseStamped getGoalPose() { return goal_pose; }

    /**
     * @brief Get the Approach Pose object
     * 
     * @return geometry_msgs::PoseStamped 
     */
    geometry_msgs::PoseStamped getApproachPose();

    /**
     * @brief Get the Grasp Pose object
     * 
     * @return geometry_msgs::PoseStamped 
     */
    geometry_msgs::PoseStamped getGraspPose();

    /**
     * @brief Get the Pose to travel
     *
     * @return geometry_msgs::PoseStamped
     */
    geometry_msgs::PoseStamped getTravelPose();

    /**
     * @brief Get the Pose to place the object
     *
     * @return geometry_msgs::PoseStamped
     */
    geometry_msgs::PoseStamped getPlacingPose();

    /**
     * @brief Function used to move the wanted group to the wanted pose
     * 
     * @param move_group 
     * @param goal 
     * @return true 
     * @return false 
     */
    bool moveToGoal(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::PoseStamped goal);

    /**
     * @brief Funcion used to grasp the target object
     * 
     * @return true 
     * @return false 
     */
    bool graspObject(); 
    
    /**
     * @brief Function to relesae the object
     * 
     * @return true 
     * @return false 
     */
    bool releaseObject();  


private:

    /**
     * @brief Convert PoseWithCovarianceStamped to geometry_msgs::PoseStamped
     * 
     * @param input 
     * @return geometry_msgs::PoseStamped 
     */
    geometry_msgs::PoseStamped ToPoseStamped(geometry_msgs::PoseWithCovarianceStamped input);
    

    //// Goal poses wrt base_footprint frame
    geometry_msgs::PoseStamped goal_pose , preliminary_goal_pose, grasp_goal_pose, travel_pose, placing_pose;

    //// ID of the target's tag
    int target_id;

    //// Specify if the service must start (0), if the sice is still mooving the arm (1)
    //// or if the movement is completed (2)
    int flag = 0;

    //// Pose of all objects
    std::vector<myTAG> detected_pose;

    //// Cols of the rotation matrix representing the rotation between tags and base_footprint
    tf2::Vector3 col1, col2, col3;

    //// Gripper functionalityes
    GripperManager gripper;
    GripperAttacher attacher;

};

