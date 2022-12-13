/**
 * @file pandp_manager.cpp
 * @brief Implementation of an action client that requests human_node the id to pick up,
 * moves Tiago to the table (in a given position/orientation) and manages nodes B and C
 *
 * @date 2022-01-20
 *
 */

// c++ packages
#include <sstream>
#include <string>
#include <cstdlib>

// Boost headers
#include <boost/shared_ptr.hpp>

// ros packages
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>

#include <actionlib/client/terminal_state.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>


//#include <tiago_iaslab_simulation/Objs.h>
#include <object_manipulation/Objs.h>

// Action for moving the robot to the table
#include <object_manipulation/MoveTiagoAction.h>

// Tag localization request
#include <object_manipulation/TagLocaliz.h>

// Arm manipulation request
#include <object_manipulation/arm_handler.h>

// OUR LIBRARY FOR TAG LOCALIZATION
#include "../include/object_manipulation/TagLocalization_lib.h"

// OUR LIBRARY FOR MOVING TIAGO TO TABLE AND ITS HEAD
#include "../include/object_manipulation/MoveTiago_lib.h"



/***************************************/
/***           FUNCTIONS             ***/
/***************************************/

// Function used to perform the frame trasformation between camera frame and base_footprint frame
// of the detected tag's poses
void changeFrame(std::vector< myTAG > new_tags, std::vector< myTAG >& tags, std::string frame_name);

// Funcion used to perform tag detection in multiple Tiago's head orientation
std::vector< myTAG > localizeTags(  int                 target_ID, 
                                    TagRequests&        tag_request, 
                                    MoveTiagoClient&  move_client, 
                                    std::string         frame_name );

/***************************************/
/***              MAIN               ***/
/***************************************/


int main (int argc, char **argv) {


    /**********************/
    // PART 0: Command Line Inputs

    std::string help = "Use: \n\n";
    help += " -h,   --help \n \n";
    help += " -hn,  --human_node  : if you want to makes the human node to select which object to pick \n\n";
    help += " -o,   --object      : if you want to select a specific object \n\nexample: -o 2 will select the second object \n\n";
    help += " -a,   --all         : if you want to pick all the objects \n\n";

    if(argc == 1 || argc > 3) {
        std::cout << help;
        return 0;
    }

    bool use_human_node = false;
    bool three_objects = false;
    std::vector<int> target_ID;

    if (argc == 2){

        auto cmd = argv[1]; 
        if(strcmp(cmd, "--help") == 0 ||strcmp(cmd, "-h") == 0 ) {
            std::cout << help;
            return 0;
        }
        else if(strcmp(cmd, "--human_node") == 0 ||strcmp(cmd, "-hn") == 0 ) {
            use_human_node = true;
        }
        else if(strcmp(cmd, "--all") == 0 ||strcmp(cmd, "-a") == 0 ) {
            use_human_node = true;
            three_objects = true;
            ROS_INFO("Starting pick and place manager for all the three objects ");
        }
        else {
            std::cerr << "\nInvalid input argument \n\n" << help;
            return 0;
        }
    }

    if (argc == 3) {

        auto cmd = argv[1];
        auto n = argv[2];
        if(strcmp(cmd, "--object") == 0 ||strcmp(cmd, "-o") == 0 ) {
            
            if (strcmp(n, "1") == 0)        target_ID.push_back(1);
            else if (strcmp(n, "2") == 0)   target_ID.push_back(2);
            else if (strcmp(n, "3") == 0)   target_ID.push_back(3);
            else{
                std::cerr << "\nInvalid input argument: specify the object ID: 1, 2 or 3 \n\n" << help;   
                return 0; 
            }
        }
        else {
            std::cerr << "\nInvalid input argument \n\n" << help;
            return 0;
        }   
        ROS_INFO("Starting pick and place manager fot object [%i]!",target_ID[0]);
    }



    /**********************/
    // PART 1: Request service to human node

    // Start the node
    ros::init(argc, argv, "pandp_manager");
    ros::NodeHandle n;

    if (use_human_node) {
        ROS_INFO("Starting pick and place manager using human_node!");  
        ros::ServiceClient human_client = n.serviceClient<object_manipulation::Objs>("/human_objects_srv");
        object_manipulation::Objs Objs_srv;

        Objs_srv.request.ready = true;
        Objs_srv.request.all_objs = three_objects;

        while (human_client.call(Objs_srv) != 1){    
            ;  // wait response from human_node
        }
        
        for ( int i = 0 ; i < Objs_srv.response.ids.size() ; i++) {
            target_ID.push_back(Objs_srv.response.ids[i]);
        }

        if (three_objects) {
            ROS_INFO("The robot is going to pick objects: %i, %i, %i", target_ID[0], target_ID[1], target_ID[2]);
        }

    }
    
    // Istance of class that permits to move Tiago
    MoveTiagoClient move_tiago;

    // Do all the action for every ID requested
    for ( int i = 0 ; i < target_ID.size() ; i++) {

        int target_object_id = target_ID[i];

        ROS_INFO("Requested object: %i", target_ID[i]);


        /******************************/
        // PART 2: Move TIAGo

        // Navigate Tiago to the preliminary position near the table, but don't do it
        // if we have to pick all the tags and we are returning after placed the obj 1
        if ( i==0 || target_object_id != 2 ) {
            move_tiago.navigatePreliminary();
        }
        
        move_tiago.navigateToTable(target_object_id);
        

        /**********************/
        // PART 3: Localize and measure pose of target object and obstacles with apriltag

        ROS_INFO("Waiting for tags locations...");
        TagRequests TagRequest(n);

        std::vector< myTAG > ALL_TAGS = localizeTags(target_object_id, TagRequest, move_tiago, "/base_footprint");

        ROS_INFO("Detected tags (pose wrt base_footprint frame): ");
        TagRequest.displayTags(ALL_TAGS);


        /**************************/
        // part4:  arm manipulation  

        // Call the picking service of the arm_handler node
        object_manipulation::arm_handler arm_srv;
        ros::ServiceClient arm_client = n.serviceClient<object_manipulation::arm_handler>("pick_srv");
        ROS_INFO("Waiting for Arm handler service to start...");
        
        std::vector<int> ALL_tags_id; 
        std::vector<geometry_msgs::PoseWithCovarianceStamped> ALL_tags_pose;

        for(int i = 0; i < ALL_TAGS.size(); i++){
            ALL_tags_id.push_back(ALL_TAGS[i].id[0]);
            ALL_tags_pose.push_back(ALL_TAGS[i].pose);
        }
        
        arm_srv.request.object_id = ALL_tags_id;
        arm_srv.request.object_pose = ALL_tags_pose;
        arm_srv.request.target_id = target_object_id;

        // Wait for the service response
        while ((arm_client.call(arm_srv) != 1)) {}
        ROS_INFO("Service started! ");
        
        while ((arm_client.call(arm_srv) != 1) || arm_srv.response.Service_done != 1) {
            ros::Duration(0.5).sleep();
            if (arm_srv.response.Service_done == -1) ROS_INFO("ERROR IN ARM HANDLER");
        }
        std::cout << std::endl;


        /**************************/
        // part5:  move TIAGo to the area where the cylindrical tables are detected

        // first, just turn on the spot in order to not to hit the table
        object_manipulation::MoveTiagoGoal original_table_goal = move_tiago.returnWantedToTablePose(target_object_id);

        if(target_object_id == 1) {
            move_tiago.navigate(original_table_goal.pos_x, original_table_goal.pos_y, 0.5);   // turn
        }

        if(target_object_id == 2) {
            move_tiago.navigate(original_table_goal.pos_x, original_table_goal.pos_y, 0.0);   // turn
        }

        if(target_object_id == 3) {
            move_tiago.navigate(original_table_goal.pos_x, original_table_goal.pos_y, 0.5);   // turn
        }

        std::vector<std::pair<double, double>>  tables_pos;
        tables_pos = move_tiago.navigateToDetection();

        if (tables_pos.size() == 0)  ROS_ERROR("No tables detected!");


        /**************************/
        // part6:  Navigate to the coloured table and place the object


        switch (target_object_id) 
        {
        case 1:
            move_tiago.navigate(tables_pos[2].first, tables_pos[2].second-1.2, 3.14159/2);
            move_tiago.navigate(tables_pos[2].first, tables_pos[2].second-0.6, 3.14159/2);
            break;
        case 2:
            move_tiago.navigate(tables_pos[1].first, tables_pos[1].second-1.2, 3.14159/2);
            move_tiago.navigate(tables_pos[1].first, tables_pos[1].second-0.6, 3.14159/2);
            break;
        case 3:
            move_tiago.navigate(tables_pos[0].first, tables_pos[0].second-1.2, 3.14159/2);
            move_tiago.navigate(tables_pos[0].first, tables_pos[0].second-0.6, 3.14159/2);
            break;
        }
        

        // Call the placing service of the arm_handler node
        ros::ServiceClient arm_placing_client = n.serviceClient<object_manipulation::arm_handler>("place_srv");
        ROS_INFO("Waiting for place service to start...");

        // Wait for the service response
        while ((arm_placing_client.call(arm_srv) != 1)) {}
        ROS_INFO("Service started!");
        
        while ((arm_placing_client.call(arm_srv) != 1) || arm_srv.response.Service_done != 1) {
            ros::Duration(0.5).sleep();
            if (arm_srv.response.Service_done == -1)  ROS_INFO("ERROR IN ARM HANDLER");
        }
        
        // If we have to return at the table to pick other objects it is convenient to return first
        // in detection position, in order to avoid to get stuck, with different orientation

        object_manipulation::MoveTiagoGoal original_detection_goal = move_tiago.returnDetectionGoal();

        if (target_ID.size() > 1 )  move_tiago.navigate(original_detection_goal.pos_x, original_detection_goal.pos_y, -1.6);
    }

    ROS_INFO("Job complete!");
    return 0;
}



void changeFrame(std::vector< myTAG > new_tags, std::vector< myTAG >& tags, std::string frame_name) {

    for (int i=0; i < new_tags.size(); i++) {

        ROS_INFO("[TF] Tag %i found", new_tags[i].id[0]);

        // Retrieve transformation from camera frame to base_footprint frame
        tf::TransformListener TFlistener;

        // Check if the tag is already present 
        bool already_in = false;
        for(int j = 0; j < tags.size(); j++) {
            if (new_tags[i].id[0] == tags[j].id[0]) already_in = true;        
        }

        if(!already_in) {

            // Get the proper tf's name
            std::string tag_frame_name = "/tag_"+std::to_string(new_tags[i].id[0]);

            ROS_INFO("[TF] Looking for tf: ");
            std::cout << tag_frame_name << std::endl;
            
            // Get the transform and Store the tag
            tf::StampedTransform tag2base_footprint;

            try {

                TFlistener.waitForTransform(frame_name, tag_frame_name, ros::Time::now(), ros::Duration(1.0));
                TFlistener.lookupTransform(frame_name, tag_frame_name, ros::Time(0), tag2base_footprint);
                static tf::TransformBroadcaster br;
                br.sendTransform(tf::StampedTransform(tag2base_footprint, ros::Time::now(), "world", tag_frame_name));

                // Overwrite the pose wrt camera frame with pose wrt base_footprint frame
                geometry_msgs::Pose msg;
                tf::poseTFToMsg(tag2base_footprint, msg);
                new_tags[i].pose.pose.pose = msg;
                
                tags.push_back(new_tags[i]);
                //std::cout << "[DEBUG] [TF] ho inserito la tag in ALL_TAGS che ora ha lunghezza: " << tags.size() << std::endl;

            }
            catch(const tf2::LookupException& e) {
                std::cerr << "[DEBUG] [TF] !!Error!! Unable to lookup the transform "  << std::endl;
            }      
        }
        else
            ROS_INFO("[TF] Tag already present!");
    }
}

std::vector< myTAG > localizeTags(  int                 target_ID, 
                                    TagRequests&        tag_request, 
                                    MoveTiagoClient&  move_client, 
                                    std::string         frame_name )
{

    // Define rpy values for the head movements
    // head_rpy[ID_case][#of_movement][x,y or z]

    std::vector<std::vector<std::vector<double>>> head_rpy;

    std::vector<std::vector<double>> blue_case 
    {
        {0.12,  0.2,   0.5},
        {0.0,   0.145, 0.5},
        {-0.07, 0.14,  0.5},
        {-0.05, 0.1,   0.5}
    };
    std::vector<std::vector<double>> green_case
    {
        {0.17, 0.2,   0.5},
        {0.1,   0.15,  0.5},
        {-0.1,  0.15,  0.5},
        {-0.07, 0.1,   0.5}
    };
    std::vector<std::vector<double>> red_case
    {
        {-0.1,  0.15,   0.5},
        {-0.15, 0.05,  0.5},
        {0.1,  0.1,  0.5},
        {0.07,  0.1,  0.5}
    };

    head_rpy.push_back(blue_case);
    head_rpy.push_back(green_case);
    head_rpy.push_back(red_case);

    // Tags with pose in camera frame, and tags with pose in base_footprint frame
    std::vector< myTAG > tags_cf, tags_bfp ;

    for(int i = 0; i<4; i++) 
    {
        // move head down 
        std::cout << std::endl;
        ROS_INFO("Moving head of TIAGo [%i]...",i+1);
        //std::cout << "[MOVE_HEAD] Moving head of TIAGo #"<< i+1 << std::endl;
        double r = head_rpy[target_ID-1][i][0];
        double p = head_rpy[target_ID-1][i][1];
        double y = head_rpy[target_ID-1][i][2];

        //std::cout << "[DEBUG] [MOVE_HEAD] r p y : "<< r << "  "<< p << "  "  << y << std::endl;
        
        move_client.moveHead(r, p, y);
        ROS_INFO( "[MOVE_HEAD] Head in position \n" );

        tags_cf = tag_request.getTags();
        ROS_INFO( "\n[TAGS] Number of detected tags:  %zu ",tags_cf.size() );
        tag_request.displayTags(tags_cf);
        changeFrame(tags_cf, tags_bfp, frame_name);
    }

    // Restore head position
    for(int i = 3; i>=0; i--) 
    {
        double r = -head_rpy[target_ID-1][i][0];
        double p = -head_rpy[target_ID-1][i][1];
        double y = head_rpy[target_ID-1][i][2];

        move_client.moveHead(r, p, y);
    }
    //ROS_INFO("Head returned!");
    ROS_INFO( "[MOVE_HEAD] Head returned in position" );

    ROS_INFO( " \n \n[TAG] Resume of all detected tags" );



    return tags_bfp;
}

    
