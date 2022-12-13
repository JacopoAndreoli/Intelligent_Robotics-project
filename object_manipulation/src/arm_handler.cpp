/**
 * @file arm_handler.cpp
 * @brief Implementation of the node that handle the manipulation of Tiago's arm, in order
 * to grasp the target object.
 * 
 * @date 2022-01-12
 * 
 * 
 */


#include "internal/ArmHandler_lib.cpp"



int main (int argc, char **argv) {

  // Init the ROS node
  ros::init(argc, argv, "arm_handler");
  ros::NodeHandle arm_h;
  ArmHandler arm_handler;

  // The two services, one for pick the object from the table and one for place the object 
  ros::ServiceServer arm_manipul_srv_pick = arm_h.advertiseService("pick_srv", &ArmHandler::handlingPICKRequestCB, &arm_handler);
  ros::ServiceServer arm_manipul_srv_place = arm_h.advertiseService("place_srv", &ArmHandler::handlingPLACERequestCB, &arm_handler);

  ROS_INFO_STREAM("Arm handler service started");

  // MoveIt istance
  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");

  // Collision objects istance
  CollisionObjects scene_objs;

  geometry_msgs::PoseStamped initial_pose;
  std::vector< double > initial_joints_values;


  while (ros::ok()){  

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // The pick service has been called, we need to move the arm to pick the object
    if(arm_handler.execute() == 1){

      // Initialization of planning scene interface
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      // Initialization of Rviz Visual Tools
      visual_tools.deleteAllMarkers();
      visual_tools.loadRemoteControl();
      visual_tools.trigger();

      // Retrieve information acquired with the callback (request items of the service)
      std::vector<myTAG> detected_pose = arm_handler.getTags();
      int target_id = arm_handler.getTargetID();
      
      // Configure and create the collision objects  
      scene_objs.configure(detected_pose, planning_scene_interface, target_id);
      bool add = scene_objs.addObjectsCollision(visual_tools);

      if(add)  std::cout << "[ARM_HANDLER] collision objects updated"  << std::endl;
      else arm_handler.setFlag(-1);
      
      // Set the planning interface and retrieve the needed goal poses
      moveit::planning_interface::MoveGroupInterface move_group("arm_torso");

      // get initial pose of arm + joint values
      initial_pose = move_group.getCurrentPose();
      initial_joints_values = move_group.getCurrentJointValues();

      geometry_msgs::PoseStamped preliminary_goal_pose, grasp_goal_pose, travel_pose;

      preliminary_goal_pose = arm_handler.getApproachPose();
      grasp_goal_pose = arm_handler.getGraspPose();
      travel_pose = arm_handler.getTravelPose();
      
      bool success;

      ROS_INFO("Going to preliminary grasp position...");
      success = arm_handler.moveToGoal(move_group, preliminary_goal_pose);
      if(!success) arm_handler.setFlag(-1);
       
      ROS_INFO("Going to grasping position...");
      success = arm_handler.moveToGoal(move_group, grasp_goal_pose);
      if(!success) arm_handler.setFlag(-1);

      // Attach object to moveit
      std::vector<std::string> touch_links;
      touch_links.push_back("gripper_link");
      touch_links.push_back("gripper_left_finger_link");
      touch_links.push_back("gripper_right_finger_link");
      touch_links.push_back("arm_7_link");
      touch_links.push_back("arm_tool_link");

      move_group.attachObject(std::to_string(target_id),"arm_7_link", touch_links);

      //scene_objs.getTargetAttached(visual_tools);     // OLD

      // Close gripper and attach object in gazebo
      success = arm_handler.graspObject();
      if(!success) arm_handler.setFlag(-1);

      ROS_INFO("Object picked: returning to preliminary grasp position");
      success = arm_handler.moveToGoal(move_group, preliminary_goal_pose);
      if(!success) arm_handler.setFlag(-1);

      ROS_INFO("Going to safe travel position");
      success = arm_handler.moveToGoal(move_group, travel_pose);
      if(!success) arm_handler.setFlag(-1);

      // The object is picked
      arm_handler.setFlag(2);

      // Remove collision objects
      scene_objs.RemoveAllObject(visual_tools);     

    }


    // The place service has been called, we need to move the arm to pick the object
    if (arm_handler.execute() == 3) {

      // Initialization of planning scene interface
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      // Initialization of Rviz Visual Tools
      visual_tools.deleteAllMarkers();
      visual_tools.loadRemoteControl();
      visual_tools.trigger();

      
      // Configure and create the collision objects
      std::vector <moveit_msgs::CollisionObject> Collision_objects_vector = scene_objs.addPlacingTable(visual_tools, 0.7);

      planning_scene_interface.addCollisionObjects(Collision_objects_vector);
      ros::Duration(2.0).sleep();
      visual_tools.trigger();

      std::cout << "[ARM_HANDLER] collision objects updated"  << std::endl;

      // Set the planning interface and retrieve the needed goal poses
      moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
      geometry_msgs::PoseStamped placing_pose, travel_pose;

      placing_pose = arm_handler.getPlacingPose();
      travel_pose = arm_handler.getTravelPose();
      
      bool success;

      ROS_INFO("Going to placing position...");
      success = arm_handler.moveToGoal(move_group, placing_pose);
      if(!success) arm_handler.setFlag(-1);
       
      // Open gripper and detach object in gazebo
      success = arm_handler.releaseObject();
      if(!success) arm_handler.setFlag(-1);
      else ROS_INFO("Object release succesfull");

      //scene_objs.getTargetDetached(visual_tools);     // OLD
      move_group.detachObject(std::to_string(arm_handler.getTargetID()));

      ROS_INFO("Going to safe travel position");
      success = arm_handler.moveToGoal(move_group, travel_pose);
      if(!success) arm_handler.setFlag(-1);
      
      ROS_INFO("Going back to initial arm position...");
      // Reset the very same joint positions
      move_group.setJointValueTarget(initial_joints_values);
      moveit::planning_interface::MoveGroupInterface::Plan planner_interface;
      move_group.plan(planner_interface);
      move_group.move();

      // The object is released
      arm_handler.setFlag(4);

      // Remove collision objects
      scene_objs.RemoveAllObject(visual_tools);
      scene_objs.RemoveTargetObject(visual_tools);

    }

    ros::Rate(20).sleep();
    ros::spinOnce();

  }

  ros::waitForShutdown();
  return 0;
}
  








