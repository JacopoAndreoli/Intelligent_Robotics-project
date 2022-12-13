/**
 * @file GripperAttacher_lib.h
 * @author group 01
 * @brief Class that uses the service provided by gazebo_ros_link_attacher to 
 * attach the object tothe gripper in Gazebo
 * 
 * @date 2022-01-15
 * 
 */

#include "ros/ros.h"
#include <string>
#include <cstdlib>
#include "gazebo_ros_link_attacher/Attach.h"


/**
 * @brief Class used to manage the gazebo_ros_link_attacher package
 * in order to attach objects to the Tiago gripper. The gripper can grasp only one object
 * at time, so you need to detach the object before attach another one.
 * 
 */
class GripperAttacher
{

public:
    GripperAttacher() 
    {
        gripper_in_use = false;
    };

    ~GripperAttacher() {};

    /**
     * @brief Attach the object to the Tiago gripper (left finger in particular)
     * 
     * @param model_name name of the model to be attached
     * @param link_name name of the link to be attached
     * @return true 
     * @return false 
     */
    bool attach(std::string model_name, std::string link_name);

    /**
     * @brief Detach the current attached object
     * 
     * @return true 
     * @return false 
     */
    bool detach();
    
private:

    ros::NodeHandle n;
    std::string model, link;
    bool gripper_in_use;
    
};


