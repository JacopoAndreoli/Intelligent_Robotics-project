#include "../include/object_manipulation/TagLocalization_lib.h"

/**************************************/
/***        fcn implementation      ***/
/**************************************/

// CONSTRUCTORS IN HEADER FILES

// Callback to activate when tag_localiz_srv is requested
bool TagLocalization::tagRequestCB(object_manipulation::TagLocaliz::Request &req, object_manipulation::TagLocaliz::Response &res) {

    this->tag_to_detect = req.object_id;

    is_tag_detected = false;

    

    for(int tag_idx=0; tag_idx < this->found_tags.size(); tag_idx++){

        if ((found_tags[tag_idx].id.size() != 0) && (found_tags[tag_idx].id[0] == tag_to_detect)){

            tag_to_detect_pose = found_tags[tag_idx].pose;
            is_tag_detected = 1;
            ROS_INFO("Found requested tag [%i]...", tag_to_detect);
        }

    }

    if(is_tag_detected == 0){
        res.object_pose.push_back(0.0);
        res.object_pose.push_back(0.0);
        res.object_pose.push_back(0.0); // position-z is 0 if no tag detected!
    }


    if(is_tag_detected == 1){
        //ROS_INFO("Target tag pose wrt cameraframe returned!");
        res.object_pose.push_back(this->tag_to_detect_pose.pose.pose.position.x);
        res.object_pose.push_back(this->tag_to_detect_pose.pose.pose.position.y);
        res.object_pose.push_back(this->tag_to_detect_pose.pose.pose.position.z);
        res.object_pose.push_back(this->tag_to_detect_pose.pose.pose.orientation.x);
        res.object_pose.push_back(this->tag_to_detect_pose.pose.pose.orientation.y);
        res.object_pose.push_back(this->tag_to_detect_pose.pose.pose.orientation.z);
        res.object_pose.push_back(this->tag_to_detect_pose.pose.pose.orientation.w);
    }

    return true;

}

void TagLocalization::tagReceivedCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {


    this->found_tags.clear();

    myTAG newtag;

    for (int tag_idx=0 ; tag_idx < msg->detections.size(); tag_idx++) {
        newtag.pose = msg->detections[tag_idx].pose;
        newtag.id.clear();
        newtag.id.push_back(msg->detections[tag_idx].id[0]);
        this->found_tags.push_back(newtag);
    }

    

    return;
}

// Add tag to detected_tags vector
void TagRequests::addTag(myTAG newTAG) {

    bool already_in = false;

    for (int tag_idx = 0; tag_idx < detected_tags.size(); tag_idx++){   // check if tag is already present

        if (detected_tags[tag_idx].id[0] == newTAG.id[0]) {
            already_in = true; 
        }
    }

    if(!already_in)  {
        detected_tags.push_back(newTAG);
    }

    return;
}

// Return detected tags
std::vector< myTAG > TagRequests::tagsRead(){

    std::vector< myTAG > temp_detected_tags = detected_tags;
    detected_tags.clear();
    return temp_detected_tags;

}

// Send to tag localizer node a service request to read tags
void TagRequests::sendRequest(){

    for (int id_idx=1; id_idx <= 7; id_idx++){

        // request service "TagLocaliz" to node tag_localization.cpp
        object_manipulation::TagLocaliz TagLocaliz_srv;

        TagLocaliz_srv.request.object_id = id_idx;

        while ((tag_client.call(TagLocaliz_srv) != 1)){    // wait response from tag_localization client
            ;
        }

        if (TagLocaliz_srv.response.object_pose[2] != 0.0) {

            myTAG found_tag;
            found_tag.id.push_back(id_idx);
            found_tag.pose.pose.pose.position.x = TagLocaliz_srv.response.object_pose[0];
            found_tag.pose.pose.pose.position.y = TagLocaliz_srv.response.object_pose[1];
            found_tag.pose.pose.pose.position.z = TagLocaliz_srv.response.object_pose[2];
            found_tag.pose.pose.pose.orientation.x = TagLocaliz_srv.response.object_pose[3];
            found_tag.pose.pose.pose.orientation.y = TagLocaliz_srv.response.object_pose[4];
            found_tag.pose.pose.pose.orientation.z = TagLocaliz_srv.response.object_pose[5];
            found_tag.pose.pose.pose.orientation.w = TagLocaliz_srv.response.object_pose[6];

            this->addTag(found_tag);
        }

    }
}

// Print out all the information about the detected tags
void TagRequests::displayTags(std::vector< myTAG > tags) {
    for (int i=0; i < tags.size(); i++) {

        double x =  tags[i].pose.pose.pose.position.x;
        double y =  tags[i].pose.pose.pose.position.y;
        double z =  tags[i].pose.pose.pose.position.z;

        double x_ = tags[i].pose.pose.pose.orientation.x;
        double y_ = tags[i].pose.pose.pose.orientation.y;
        double z_ = tags[i].pose.pose.pose.orientation.z;
        double w_ = tags[i].pose.pose.pose.orientation.w;

        ROS_INFO("Pose of tag [%i] wrt given frame:\n [x:%f y:%f z:%f, x:%f y:%f z:%f w:%f]", tags[i].id[0], x, y, z, x_, y_, z_, w_);

    }
}
 
// Perform both sendRequest() and tagsRead() in one call
std::vector< myTAG > TagRequests::getTags() {

    this->sendRequest();
    std::vector< myTAG > output = this->tagsRead();
    return output;

}
