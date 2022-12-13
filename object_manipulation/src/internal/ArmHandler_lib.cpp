#include "../include/object_manipulation/ArmHandler_lib.h"


bool ArmHandler::handlingPICKRequestCB(object_manipulation::arm_handler::Request &req, object_manipulation::arm_handler::Response &res)
{

    // First call of the service
    if(flag == 0) 
    {
        // all detected objects  id
        std::vector<int> ALL_id = req.object_id;

        // all detected objects pose
        std::vector<geometry_msgs::PoseWithCovarianceStamped> ALL_pose = req.object_pose;

        // target ID
        target_id = req.target_id;

        // variable use to remove the collision object after the preliminary motion
        myTAG target_pose;
        //use as a container variable creating detected_pose vector
        myTAG obstacle;

        detected_pose.clear();

        for(int i=0; i < ALL_id.size(); i++){

            if(ALL_id[i] == target_id){
            //saving target myTAG
            target_pose.id.push_back(ALL_id[i]);
            target_pose.pose = ALL_pose[i];
            //saving all detected myTAG
            obstacle.id.push_back(ALL_id[i]); // here the variable obstacle is used as container
            obstacle.pose = ALL_pose[i];
            detected_pose.push_back(obstacle);
            obstacle.id.clear();
            
            }
            else{
            //continue to save all detected myTAG
            obstacle.id.push_back(ALL_id[i]);
            obstacle.pose = ALL_pose[i];
            detected_pose.push_back(obstacle);
            obstacle.id.clear();
            }

        }

        goal_pose = ToPoseStamped(target_pose.pose);

        // SET FLAG TO 1, PICKING ROUTINE IS READY AND WILL BE EXECUTED!
        flag = 1;
        res.Service_done = 0;
        return true;
    }

    // The service is already performing arm movements
    if( flag == 1 ) 
    {
        res.Service_done = 0;
        return true;
    }

    // The object is picked
    if( flag == 2 ) 
    {
        res.Service_done = 1;
        return true;
    }
    
    // Error state
    if( flag == -1 ) 
    {
        res.Service_done = -1;
        return true;
    }

}

bool ArmHandler::handlingPLACERequestCB(object_manipulation::arm_handler::Request &req, object_manipulation::arm_handler::Response &res)
{

    //ROS_INFO("Release service called, actual flag state: %i", flag);
    // First call of the service
    if( flag == 2 ) 
    {
        flag = 3;
        res.Service_done = 0;
        return true;
    }
    
    // The service is already performing arm movements
    if( flag == 3 ) 
    {
        res.Service_done = 0;
        return true;
    }
    
     // The object is placed
    if( flag == 4 ) 
    {
        flag = 0;
        res.Service_done = 1;
        return true;
    }
    
    res.Service_done = 0;
    return true;
    // Error state
    if( flag == -1 ) 
    {
        res.Service_done = -1;
        return true;
    }
}

geometry_msgs::PoseStamped ArmHandler::getApproachPose() 
{
    tf2::Quaternion q_orig, q_wantedArm_wrt_object;
    if (target_id == 2){  // if green, take from above (z axis of map, basically)
        q_wantedArm_wrt_object.setRPY(1.570795, 1.570795/2, 1.570795);
    }
    else{                 // otherwise, take from top of april tag
        q_wantedArm_wrt_object.setRPY(0.0, 1.570795, 0.0);
    }

    // retrieve rotation matrix of object wrt base_footprint
    tf2::convert(goal_pose.pose.orientation , q_orig);
    tf2::Matrix3x3 m(q_orig);
    col3 = m.getColumn(2);
    col2 = m.getColumn(1);
    col1 = m.getColumn(0);


    if (target_id == 2){  // if green
        float n = 0.23;  // we want to move ~20 cm above the pose recovered through the detection
        preliminary_goal_pose = goal_pose;
        preliminary_goal_pose.pose.position.x = goal_pose.pose.position.x + col3.getX()*n - col2.getX()*n;
        preliminary_goal_pose.pose.position.y = goal_pose.pose.position.y + col3.getY()*n - col2.getY()*n;
        preliminary_goal_pose.pose.position.z = goal_pose.pose.position.z + col3.getZ()*n - col2.getZ()*n;
    }
    else{                 // if red or blue
        float n = 0.30;  // we want to move ~30 cm above the pose recovered through the detection
        preliminary_goal_pose = goal_pose;
        preliminary_goal_pose.pose.position.x = goal_pose.pose.position.x + col3.getX()*n;
        preliminary_goal_pose.pose.position.y = goal_pose.pose.position.y + col3.getY()*n;
        preliminary_goal_pose.pose.position.z = goal_pose.pose.position.z + col3.getZ()*n;
    }

    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);
    // 1.570795
    // Rotate the previous pose by +90° about Y: the gripper must point to the tag
    tf2::Quaternion q_wantedArm_wrt_bfp = q_orig*q_wantedArm_wrt_object;
    preliminary_goal_pose.pose.orientation.x = q_wantedArm_wrt_bfp[0];
    preliminary_goal_pose.pose.orientation.y = q_wantedArm_wrt_bfp[1];
    preliminary_goal_pose.pose.orientation.z = q_wantedArm_wrt_bfp[2];
    preliminary_goal_pose.pose.orientation.w = q_wantedArm_wrt_bfp[3];

    return preliminary_goal_pose;

}

geometry_msgs::PoseStamped ArmHandler::getGraspPose() 
{
    if (target_id == 2){  // if green
        float n_grasp_z = 0.15;  // approach
        float n_grasp_y = 0.163;  // approach
        grasp_goal_pose = preliminary_goal_pose;
        grasp_goal_pose.pose.position.x = goal_pose.pose.position.x + col3.getX()*n_grasp_z - col2.getX()*n_grasp_y;
        grasp_goal_pose.pose.position.y = goal_pose.pose.position.y + col3.getY()*n_grasp_z - col2.getY()*n_grasp_y;
        grasp_goal_pose.pose.position.z = goal_pose.pose.position.z + col3.getZ()*n_grasp_z - col2.getZ()*n_grasp_y;
    }
    else{                 // if red or blue
        float n_grasp = 0.18;  // we want to move ~16 cm above the pose recovered through the detection
        if (target_id == 3) //if red
            n_grasp = 0.205; // give some cm more - the red cube is small
        grasp_goal_pose = preliminary_goal_pose;
        grasp_goal_pose.pose.position.x = goal_pose.pose.position.x + col3.getX()*n_grasp;
        grasp_goal_pose.pose.position.y = goal_pose.pose.position.y + col3.getY()*n_grasp;
        grasp_goal_pose.pose.position.z = goal_pose.pose.position.z + col3.getZ()*n_grasp;
    }
    
    return grasp_goal_pose;
}

geometry_msgs::PoseStamped ArmHandler::getTravelPose()
{
    if (target_id == 2)
        travel_pose.pose.position.x = preliminary_goal_pose.pose.position.x-0.18;
    else
        travel_pose.pose.position.x = preliminary_goal_pose.pose.position.x-0.24;   // reduce a little bit x
    travel_pose.pose.position.y = preliminary_goal_pose.pose.position.y;
    travel_pose.pose.position.z = preliminary_goal_pose.pose.position.z;
    travel_pose.pose.orientation.x = preliminary_goal_pose.pose.orientation.x;
    travel_pose.pose.orientation.y = preliminary_goal_pose.pose.orientation.y;
    travel_pose.pose.orientation.z = preliminary_goal_pose.pose.orientation.z;
    travel_pose.pose.orientation.w = preliminary_goal_pose.pose.orientation.w;

    return travel_pose;

}

geometry_msgs::PoseStamped ArmHandler::getPlacingPose()
{

    placing_pose.pose.position.x = 0.75; // take the object to the table
    placing_pose.pose.position.y = 0;
    placing_pose.pose.position.z = 1.02; // take the object to the table
    placing_pose.pose.orientation.x = travel_pose.pose.orientation.x;
    placing_pose.pose.orientation.y = travel_pose.pose.orientation.y;
    placing_pose.pose.orientation.z = travel_pose.pose.orientation.z;
    placing_pose.pose.orientation.w = travel_pose.pose.orientation.w;

    return placing_pose;

}

geometry_msgs::PoseStamped ArmHandler::ToPoseStamped(geometry_msgs::PoseWithCovarianceStamped input)
{

    geometry_msgs::PoseStamped output;
    output.pose.position.x = input.pose.pose.position.x;
    output.pose.position.y = input.pose.pose.position.y;
    output.pose.position.z = input.pose.pose.position.z;
    output.pose.orientation.x = input.pose.pose.orientation.x;
    output.pose.orientation.y = input.pose.pose.orientation.y;
    output.pose.orientation.z = input.pose.pose.orientation.z;
    output.pose.orientation.w = input.pose.pose.orientation.w;

    return output;

}

bool ArmHandler::moveToGoal(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::PoseStamped goal) 
{
    move_group.setPlannerId("SBLkConfigDefault");
    move_group.setPoseReferenceFrame("base_footprint");
    move_group.setPlanningTime(10.0);
    

    //initialization of the planner 
    move_group.setPoseTarget(goal.pose);
    move_group.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan planner_interface;
    bool success = bool(move_group.plan(planner_interface));
    
    if(!success){
        std::cout << "[ARM_HANDLER] Impossible to find a plan" <<  std::endl;
        return false;
    }
    std::cout << "[ARM_HANDLER] plan found in " << planner_interface.planning_time_ << "seconds" << std::endl;

    ros::Time start = ros::Time::now();

    move_group.move();
    
    moveit::planning_interface::MoveItErrorCode e = move_group.move();
    if(!bool(e)){
        std::cout << "[DEBUG][ARM_HANDLER] Error in motion execution" << std::endl;
        return false;
    }
    std::cout << "[ARM_HANDLER] Motion successful: duration " << ((ros::Time::now()-start).toSec()) << "seconds" << std::endl;

    return true;

}

bool ArmHandler::graspObject() 
{
    ROS_INFO("[ARM_HANDLER] grasping object...");

    std::string model_name, link_name;

    if(target_id == 1) 
    {
        model_name = "Hexagon";
        link_name = "Hexagon_link";
        std::cout << "...blue" << std::endl;
    }
    else if(target_id == 2)
    {
    model_name = "Triangle";
    link_name = "Triangle_link";
    std::cout << "...green" << std::endl;
    }
    else if(target_id == 3)
    {
    model_name = "cube";
    link_name = "cube_link";
    std::cout << "...red" << std::endl;
    }
    else
    return false;

    gripper.closeGripper();
    return attacher.attach(model_name, link_name);
}


bool ArmHandler::releaseObject() 
{
    bool success = attacher.detach();
    gripper.openGripper();

    return success; 
} 
