#include "../include/object_manipulation/GripperAttacher_lib.h"



bool GripperAttacher::attach(std::string model_name, std::string link_name)
{
    // If something is already attached to the gripper, return
    if(gripper_in_use)
    {
        ROS_ERROR("Gripper is already attached with some object: detach first!");
        return false;
    }

    // Call the service
    ros::ServiceClient client = n.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    gazebo_ros_link_attacher::Attach srv;

    srv.request.model_name_1 =  "tiago";
    srv.request.link_name_1  =  "gripper_left_finger_link";

    srv.request.model_name_2 =  model_name;
    srv.request.link_name_2  =  link_name;

    if (client.call(srv))
    {
        // Store the attached object 
        gripper_in_use = true;
        model = model_name;
        link = link_name;
        return srv.response.ok; 
    }
    
    else  return false;
}

/**
 * @brief Detach the current attached object
 * 
 * @return true 
 * @return false 
*/

bool GripperAttacher::detach()
{
    // If there isn't any object attached to the gripper, return
    if(!gripper_in_use)
    {
        ROS_ERROR("Nothing to detach!");
        return true;
    }

    // Call the service
    ros::ServiceClient client = n.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");
    gazebo_ros_link_attacher::Attach srv;

    srv.request.model_name_1 =  "tiago";
    srv.request.link_name_1  =  "gripper_left_finger_link";

    srv.request.model_name_2 =  model;
    srv.request.link_name_2  =  link;

    // Restore and return
    if (client.call(srv) && srv.response.ok) 
    {
        gripper_in_use = false;
        return srv.response.ok; 
    }
    else  return false;
}
