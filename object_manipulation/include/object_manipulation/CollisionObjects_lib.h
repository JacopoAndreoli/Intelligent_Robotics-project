/**
 * @file CollisionObjects_lib.h
 * @author group 01
 * @brief Class that manage the collision objects, such as construction, 
 * initialization in the world and the deletion
 * 
 * @date 2022-01-12
 * 
 */

// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include "../include/object_manipulation/arm_handler.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <typeinfo>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

/**
 * @brief Structure of apriltag
 * 
 */
typedef struct {
  geometry_msgs::PoseWithCovarianceStamped pose;
  std::vector<int> id;
} myTAG;


/**
 * @brief Class used to handle the moveit collision objects, in the specific case of the objects 
 * located in the ias_lab room simulation, where objects are identified throught an april tag.
 * An additional objetcs representing the table has been added.
 * 
*/
class CollisionObjects
{

public:

    
    CollisionObjects() {};

    ~CollisionObjects() {};

    /**
     * @brief Set the parameters needed for the creation of the collision objects
     * 
     * @param tags_ 
     * @param planning_scene_interface_ 
     * @param target_ID 
     */
    void configure(std::vector<myTAG>& tags_,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_,
                    int target_ID);

    /**
     * @brief Method used to add collision objets to the scene, according to the tag identification
     * 
     * @return true 
     * @return false 
    */
    bool addObjectsCollision(moveit_visual_tools::MoveItVisualTools& visual_tools);

    /**
     * @brief Method used to remove collision objets to the scene, according to the tag identification
     * 
     * @param visual_tools 
     */
    void RemoveTargetObject(moveit_visual_tools::MoveItVisualTools visual_tools);

    /**
    * @brief remove all collision objects of the scene; exception for target object
    *
    * @return void
    */
    void RemoveAllObject(moveit_visual_tools::MoveItVisualTools visual_tools);


    /**
     * @brief Method used to add collision object of the coloured cylindrical table
     * 
     * @return std::vector <moveit_msgs::CollisionObject>
    */
    std::vector <moveit_msgs::CollisionObject> addPlacingTable(moveit_visual_tools::MoveItVisualTools& visual_tools, double distance);


    /**
    * @brief get detached the target object on rviz
    *
    * @return void
    */
    void getTargetDetached(moveit_visual_tools::MoveItVisualTools visual_tools);


private:
    
    /**
     * @brief Add the blue hexagon to the scene
     * 
     * @return moveit_msgs::CollisionObject 
    */
    moveit_msgs::CollisionObject addBlueHexagon(myTAG BHexPose);

    /**
     * @brief Add the green triangle to the scene
     * 
     * @return moveit_msgs::CollisionObject 
    */
    moveit_msgs::CollisionObject addGreenTriangle(myTAG GTrianglePose );

    /**
     * @brief Add the red cube to the scene
     * 
     * @return moveit_msgs::CollisionObject 
     */
    moveit_msgs::CollisionObject addRedCube(myTAG RCubePose);

    /**
     * @brief Add the gold hexagon to the scene
     * 
     * @return moveit_msgs::CollisionObject 
     */
    moveit_msgs::CollisionObject addGoldObstacle(myTAG GHexPose);

    /**
     * @brief Add the table to the scene according to tags location
     * 
     * @return moveit_msgs::CollisionObject 
     */
    moveit_msgs::CollisionObject addTable();

    /**
     * @brief attach the object to the end-effector
     * 
     * @return void 
     */
    void getTargetAttached(moveit_visual_tools::MoveItVisualTools visual_tools);


    //// Vector of tag containers
    std::vector<myTAG> tags;

    //// Moveit istance
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //// Target tag's ID
    int target_id;
};


