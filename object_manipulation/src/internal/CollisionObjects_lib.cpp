/**
 * @file CollisionObjects.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-01-12
 * 
 * @copyright Copyright (c) 2022
 * 
*/
#include "../include/object_manipulation/CollisionObjects_lib.h"

void CollisionObjects::configure(std::vector<myTAG>& tags_,
                  moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_,
                  int target_ID) 
{   
    target_id = target_ID;
    tags = tags_;
    planning_scene_interface = planning_scene_interface_;
}

bool CollisionObjects::addObjectsCollision(moveit_visual_tools::MoveItVisualTools& visual_tools)
{

    moveit_msgs::CollisionObject addingObjects; 
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::cout << "[ARM_HANDLER] adding table into the world"  << std::endl;

    // remove possible previous collision objects
    collision_objects.clear();


    addingObjects = addTable();
    collision_objects.push_back(addingObjects);
  
    for(int i = 0; i < tags.size(); i++){


        if(tags[i].id[0] == 1){

        addingObjects = addBlueHexagon(tags[i]);
        collision_objects.push_back(addingObjects);
        std::cout << "[ARM_HANDLER] adding blue hexagon into the world"  << std::endl;
        }
        else if(tags[i].id[0] == 2){
        addingObjects = addGreenTriangle(tags[i]);
        std::cout << "[ARM_HANDLER] adding green triangle into the world"  << std::endl;
        collision_objects.push_back(addingObjects);
        }
        else if(tags[i].id[0] == 3){
        addingObjects = addRedCube(tags[i]);
        std::cout << "[ARM_HANDLER] adding red cube into the world"  << std::endl;
        collision_objects.push_back(addingObjects);
        }
        else{
        addingObjects = addGoldObstacle(tags[i]);
        collision_objects.push_back(addingObjects);
        std::cout << "[ARM_HANDLER] adding gold obstacle into the world: id " << tags[i].id[0] << std::endl;
        }

    }

    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(2.0).sleep(); //very important; recall to add when the publisher send the message
    visual_tools.trigger();
    collision_objects.clear();

    return true;
}

moveit_msgs::CollisionObject CollisionObjects::addBlueHexagon(myTAG BHexPose){

  moveit_msgs::CollisionObject cylinder_object;

  cylinder_object.id = std::to_string(BHexPose.id[0]); // we define each obstacle as a number
  cylinder_object.header.frame_id = "base_footprint"; // the reference frame that we use for the detection
  cylinder_object.header.stamp = ros::Time::now();
  shape_msgs::SolidPrimitive object;
  object.type = object.CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[1] = 0.03;   // radius
  object.dimensions[0] = 0.10;    // lenght
  geometry_msgs::Pose pose;
  pose.position.x = BHexPose.pose.pose.pose.position.x;
  pose.position.y = BHexPose.pose.pose.pose.position.y;
  pose.position.z = BHexPose.pose.pose.pose.position.z- object.dimensions[0]/2;
  pose.orientation.x = BHexPose.pose.pose.pose.orientation.x;
  pose.orientation.y = BHexPose.pose.pose.pose.orientation.y;
  pose.orientation.z = BHexPose.pose.pose.pose.orientation.z;
  pose.orientation.w = BHexPose.pose.pose.pose.orientation.w;
  cylinder_object.primitives.push_back(object);
  cylinder_object.primitive_poses.push_back(pose);
  cylinder_object.operation = cylinder_object.ADD;

  return cylinder_object;

}

moveit_msgs::CollisionObject CollisionObjects::addRedCube(myTAG RCubePose){

  moveit_msgs::CollisionObject cube_object;

  cube_object.id = std::to_string(RCubePose.id[0]); // we define each obstacle as a number

  cube_object.header.frame_id = "base_footprint"; // the reference frame that we use for the detection
  cube_object.header.stamp = ros::Time::now();
  shape_msgs::SolidPrimitive object;
  object.type = object.BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = 0.06;   // size_x
  object.dimensions[1] = 0.06;   // size_y
  object.dimensions[2] = 0.05;   // size_z
  geometry_msgs::Pose pose;
  pose.position.x = RCubePose.pose.pose.pose.position.x;
  pose.position.y = RCubePose.pose.pose.pose.position.y;
  pose.position.z = RCubePose.pose.pose.pose.position.z- object.dimensions[1]/2;
  pose.orientation.x = RCubePose.pose.pose.pose.orientation.x;
  pose.orientation.y = RCubePose.pose.pose.pose.orientation.y;
  pose.orientation.z = RCubePose.pose.pose.pose.orientation.z;
  pose.orientation.w = RCubePose.pose.pose.pose.orientation.w;
  cube_object.primitives.push_back(object);
  cube_object.primitive_poses.push_back(pose);
  cube_object.operation = cube_object.ADD;

  return cube_object;

}

moveit_msgs::CollisionObject CollisionObjects::addGreenTriangle(myTAG GTrianglePose ){

  moveit_msgs::CollisionObject triangle_object; // the approximation of the green trinagle is done through a cube 

  triangle_object.id = std::to_string(GTrianglePose.id[0]); // we define each obstacle as a number
  triangle_object.header.frame_id = "base_footprint"; // the reference frame that we use for the detection
  triangle_object.header.stamp = ros::Time::now();
  shape_msgs::SolidPrimitive object; 
  object.type = object.BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = 0.06;   // size_x
  object.dimensions[1] = 0.07;   // size_y
  object.dimensions[2] = 0.06;   // size_z
  geometry_msgs::Pose pose;
  pose.position.x = GTrianglePose.pose.pose.pose.position.x;
  pose.position.y = GTrianglePose.pose.pose.pose.position.y;
  pose.position.z = GTrianglePose.pose.pose.pose.position.z- object.dimensions[2]/2;
  pose.orientation.x = GTrianglePose.pose.pose.pose.orientation.x;
  pose.orientation.y = GTrianglePose.pose.pose.pose.orientation.y;
  pose.orientation.z = GTrianglePose.pose.pose.pose.orientation.z;
  pose.orientation.w = GTrianglePose.pose.pose.pose.orientation.w;
  triangle_object.primitives.push_back(object);
  triangle_object.primitive_poses.push_back(pose);
  triangle_object.operation = triangle_object.ADD;

  return triangle_object;

}

moveit_msgs::CollisionObject CollisionObjects::addGoldObstacle(myTAG GHexPose){

  moveit_msgs::CollisionObject cylinder_object; // the approximation of the green trinagle is done through a cube 

  cylinder_object.id = std::to_string(GHexPose.id[0]); // we define each obstacle as a number

  cylinder_object.header.frame_id = "base_footprint"; // the reference frame that we use for the detection
  cylinder_object.header.stamp = ros::Time::now();
  shape_msgs::SolidPrimitive object;
  object.type = object.CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[1] = 0.06;   // radius
  double additional_height = 0.02;
  object.dimensions[0] = 0.20 + additional_height;   // height

  geometry_msgs::Pose pose;
  pose.position.x = GHexPose.pose.pose.pose.position.x;
  pose.position.y = GHexPose.pose.pose.pose.position.y;
  pose.position.z = GHexPose.pose.pose.pose.position.z - object.dimensions[0]/2 + additional_height;
  pose.orientation.x = GHexPose.pose.pose.pose.orientation.x;
  pose.orientation.y = GHexPose.pose.pose.pose.orientation.y;
  pose.orientation.z = GHexPose.pose.pose.pose.orientation.z;
  pose.orientation.w = GHexPose.pose.pose.pose.orientation.w;
  cylinder_object.primitives.push_back(object);
  cylinder_object.primitive_poses.push_back(pose);
  cylinder_object.operation = cylinder_object.ADD;

  return cylinder_object;

}

moveit_msgs::CollisionObject CollisionObjects::addTable( ){

  moveit_msgs::CollisionObject table_collision_object;

  table_collision_object.id = "table"; // we define each obstacle as a number

    // Retrieve position of map wrt base_footprint

    tf::TransformListener TFlistener;
    tf::StampedTransform transform;
    //ROS_INFO("Waiting for frames transformation...");
    TFlistener.waitForTransform("/base_footprint", "/map", ros::Time::now(), ros::Duration(4.0));
    TFlistener.lookupTransform("/base_footprint", "/map", ros::Time(0), transform);

    // pose of map wrt TIAGO
    double tiago_pos_x, tiago_pos_y, tiago_pos_z;
    tf2::Quaternion rotation_quaternion;

    tiago_pos_x = transform.getOrigin().x();
    tiago_pos_y = transform.getOrigin().y();
    tiago_pos_z = transform.getOrigin().z();

    rotation_quaternion[2] = transform.getRotation().z();
    rotation_quaternion[1] = transform.getRotation().y();
    rotation_quaternion[0] = transform.getRotation().x();
    rotation_quaternion[3] = transform.getRotation().w();

    tf2::Matrix3x3 m(rotation_quaternion);

    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);


  table_collision_object.header.frame_id = "base_footprint"; // the reference frame that we use for the detection
  table_collision_object.header.stamp = ros::Time::now();
  shape_msgs::SolidPrimitive object;
  object.type = object.BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = 1.05;
  object.dimensions[1] = 1.05;
  object.dimensions[2] = 0.78;


  geometry_msgs::Pose pose_wrt_map;
  pose_wrt_map.position.x = 7.8;
  pose_wrt_map.position.y = -2.983;
  pose_wrt_map.position.z = object.dimensions[2]/2 + 0.0;

  geometry_msgs::Pose pose;

  pose.position.x = cos(yaw)*pose_wrt_map.position.x - sin(yaw)*pose_wrt_map.position.y + tiago_pos_x;
  pose.position.y = sin(yaw)*pose_wrt_map.position.x + cos(yaw)*pose_wrt_map.position.y + tiago_pos_y;
  pose.position.z = pose_wrt_map.position.z + tiago_pos_z;
  pose.orientation.x = rotation_quaternion[0];
  pose.orientation.y = rotation_quaternion[1];
  pose.orientation.z = rotation_quaternion[2];
  pose.orientation.w = rotation_quaternion[3];
  table_collision_object.primitives.push_back(object);
  table_collision_object.primitive_poses.push_back(pose);
  table_collision_object.operation = table_collision_object.ADD;

  return table_collision_object;

}

void CollisionObjects::RemoveTargetObject(moveit_visual_tools::MoveItVisualTools visual_tools){

  std::vector<std::string> object_ids;
  object_ids.push_back(std::to_string(target_id));

  std::cout << "[ARM_HANDLER] removing target object from the world"  << std::endl;
  planning_scene_interface.removeCollisionObjects(object_ids);
  ros::Duration(2.0).sleep(); //very important; recall to add when the publisher send the message
  visual_tools.trigger();
  return;
}

void CollisionObjects::getTargetAttached(moveit_visual_tools::MoveItVisualTools visual_tools){

moveit_msgs::AttachedCollisionObject target_to_attach;

target_to_attach.object.id = std::to_string(target_id);
target_to_attach.link_name = "arm_tool_link";
target_to_attach.object.operation = target_to_attach.object.ADD;
planning_scene_interface.applyAttachedCollisionObject(target_to_attach); 
ros::Duration(2.0).sleep(); //very important; recall to add when the publisher send the message
visual_tools.trigger();
std::cout << "[ARM_HANDLER] attached object added to the scene" << std::endl;

}

void CollisionObjects::RemoveAllObject(moveit_visual_tools::MoveItVisualTools visual_tools){

  std::vector<std::string> object_list;
  std::vector<std::string> objects_to_remove;

  object_list = planning_scene_interface.getKnownObjectNames();

  std::cout << "[ARM_HANDLER] removing collision objects from the world"  << std::endl;


  for(int i = 0; i < object_list.size(); i++){
    if(object_list[i] != std::to_string(target_id)){
      objects_to_remove.push_back(object_list[i]);
    }
  }

  planning_scene_interface.removeCollisionObjects(objects_to_remove);
  ros::Duration(2.0).sleep(); //very important; recall to add when the publisher send the message
  visual_tools.trigger();
  return;

}

std::vector <moveit_msgs::CollisionObject> CollisionObjects::addPlacingTable(moveit_visual_tools::MoveItVisualTools& visual_tools, double distance){


  moveit_msgs::CollisionObject place_table;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  std::cout << "[ARM_HANDLER] adding placing table into the world"  << std::endl;

  place_table.id = "place_table"; 

  place_table.header.frame_id = "base_footprint"; 
  place_table.header.stamp = ros::Time::now();
  shape_msgs::SolidPrimitive object;
  object.type = object.CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[1] = 0.27;   // radius
  object.dimensions[0] = 0.695;   // height

  geometry_msgs::Pose pose;
  pose.position.x = distance;
  pose.position.y = 0.0;
  pose.position.z = object.dimensions[0]/2;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  place_table.primitives.push_back(object);
  place_table.primitive_poses.push_back(pose);
  place_table.operation = place_table.ADD;

  // add also some walls to constraint the arm
  moveit_msgs::CollisionObject wall_l, wall_r;
  wall_l.id = "wall_left";
  wall_r.id = "wall_right";

  wall_l.header.frame_id = "base_footprint";
  wall_r.header.frame_id = "base_footprint";
  wall_l.header.stamp = ros::Time::now();
  wall_r.header.stamp = ros::Time::now();
  object.type = object.BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = 2.0;   // size_x
  object.dimensions[1] = 0.1;   // size_y
  object.dimensions[2] = 2.0;   // size_z

  pose.position.x = 0;
  pose.position.y = 0.8;
  pose.position.z = object.dimensions[0]/2;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;

  wall_l.primitives.push_back(object);
  wall_l.primitive_poses.push_back(pose);
  wall_l.operation = wall_l.ADD;
  wall_r.primitives.push_back(object);
  pose.position.y = -1.0;
  wall_r.primitive_poses.push_back(pose);
  wall_r.operation = wall_r.ADD;

  //std::cout << "[DEBUG] before pushback the collision object  collision_objects size: "<< collision_objects.size()  << std::endl;
  collision_objects.push_back(place_table);
  collision_objects.push_back(wall_l);
  collision_objects.push_back(wall_r);
  //std::cout << "[DEBUG] after pushback the collision object"  << std::endl;

  return collision_objects;

}

void CollisionObjects::getTargetDetached(moveit_visual_tools::MoveItVisualTools visual_tools){

    moveit_msgs::AttachedCollisionObject target_to_detach;

    target_to_detach.object.id = std::to_string(target_id);
    target_to_detach.link_name = "arm_tool_link";
    target_to_detach.object.operation = target_to_detach.object.REMOVE;
    planning_scene_interface.applyAttachedCollisionObject(target_to_detach);
    ros::Duration(2.0).sleep(); //very important; recall to add when the publisher send the message
    visual_tools.trigger();
    std::cout << "[ARM_HANDLER] attached object added to the scene" << std::endl;

}


