#include "../include/object_manipulation/GripperManager_lib.h"
/**
 * @brief Class used to handle the gripper. In particular, this class, provides two methods,
 * one for open the gripper, and another to close it.
 * 
*/


void GripperManager::openGripper() 
{
    // Generates the goal for the TIAGo's gripper
    control_msgs::FollowJointTrajectoryGoal gripper_goal;
    openGripperGoal(gripper_goal);

    // Sends the command to start the given trajectory 
    gripper_goal.trajectory.header.stamp = ros::Time::now();
    gripper_client->sendGoal(gripper_goal);

    while(!(gripper_client->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); 
    }
    ROS_INFO("Gripper opened");
}

/**
 * @brief Function that close the gripper
 * 
 */
void GripperManager::closeGripper() 
{
    // Generates the goal for the TIAGo's gripper
    control_msgs::FollowJointTrajectoryGoal gripper_goal;
    closeGripperGoal(gripper_goal);

    // Sends the command to start the given trajectory 1s from now
    gripper_goal.trajectory.header.stamp = ros::Time::now();
    gripper_client->sendGoal(gripper_goal);

    while(!(gripper_client->getState().isDone()) && ros::ok())
    {
        ros::Duration(1).sleep(); // sleep for four seconds
    }
    ROS_INFO("Gripper closed");
}


/**
 * @brief Create a ROS action client to move TIAGo's gripper
 * 
 * @param action_client 
 * @param arm_controller_name 
*/

void GripperManager::createGripperClient(arm_control_client_Ptr& action_client, const std::string arm_controller_name)
{
    ROS_INFO("Creating action client to %s ...", arm_controller_name.c_str());

    std::string action_client_name = "/" + arm_controller_name + "/follow_joint_trajectory";
    action_client.reset( new arm_control_client(action_client_name) );

    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !action_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
    }

    if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

/**
 * @brief Generate the trajectory points for the gripper movements
 * 
 * @param goal 
 * @param l left finger displacement
 * @param r right finger displacement
*/

void GripperManager::waypointsArmGoal(control_msgs::FollowJointTrajectoryGoal& goal, double l, double r)
{
    // One waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // Trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(2);
    goal.trajectory.points[index].positions[0] = r;
    goal.trajectory.points[index].positions[1] = l;

    // Velocities
    goal.trajectory.points[index].velocities.resize(2);
    for (int j = 0; j < 2; ++j)
    {
    goal.trajectory.points[index].velocities[j] = 1.0;
    }

    goal.trajectory.points[index].time_from_start = ros::Duration(1.0);

}

/**
 * @brief Function used to generate trajectory for close the gripper
 * 
 * @param goal 
*/

void GripperManager::closeGripperGoal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");

    waypointsArmGoal(goal, 0.0, 0.0);
}

/**
 * @brief Function used to generate trajectory for open the gripper
 * 
 * @param goal 
*/

void GripperManager::openGripperGoal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");

    waypointsArmGoal(goal, 0.045, 0.045);
}








