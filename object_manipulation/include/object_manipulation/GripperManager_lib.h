/**
 * @file GripperManager_lib.h
 * @author group 01
 * @brief Class that performs the two fundamental gripper operation: open and close the gripper
 * 
 * @date 2022-01-15
 * 
 */


// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

// Action interface type for moving  joints, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


/**
 * @brief Class used to handle the gripper. In particular, this class, provides two methods,
 * one for open the gripper, and another to close it.
 * 
 */
class GripperManager
{

public:

    GripperManager()
    {
        createGripperClient(gripper_client, "gripper_controller");
    }

    ~GripperManager() {};

     /**
     * @brief Function that open the gripper
     * 
     */
    void openGripper(); 
    

    /**
     * @brief Function that close the gripper
     * 
     */
    void closeGripper();
    

private:

    /**
     * @brief Create a ROS action client to move TIAGo's gripper
     * 
     * @param action_client 
     * @param arm_controller_name 
     */
    void createGripperClient(arm_control_client_Ptr& action_client, const std::string arm_controller_name);
    

    /**
     * @brief Generate the trajectory points for the gripper movements
     * 
     * @param goal 
     * @param l left finger displacement
     * @param r right finger displacement
     */
    void waypointsArmGoal(control_msgs::FollowJointTrajectoryGoal& goal, double l, double r);
    

    /**
     * @brief Function used to generate trajectory for close the gripper
     * 
     * @param goal 
     */
    void closeGripperGoal(control_msgs::FollowJointTrajectoryGoal& goal);
    

    /**
     * @brief Function used to generate trajectory for open the gripper
     * 
     * @param goal 
     */
    void openGripperGoal(control_msgs::FollowJointTrajectoryGoal& goal);
    
    arm_control_client_Ptr gripper_client;

};







