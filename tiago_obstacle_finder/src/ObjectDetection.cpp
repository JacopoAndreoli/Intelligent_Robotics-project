#include "../include/tiago_obstacle_finder/ObjectDetection.h"

ObjectDetection::ObjectDetection()
{
    scan_request = false;
    scan_received = false;
}



void ObjectDetection::LaserCallback(const sensor_msgs::LaserScanConstPtr& msg){
    
    // If the measurements are not needed, exit.
    if(!scan_request) return;

    point_distances.clear();
    angle_measurements.clear();

    for(int i = 22; i < msg->ranges.size() - 23; i++)
        point_distances.push_back(msg->ranges[i]);

    // Get the angles vector
    generateAnglesVector( msg->angle_max, msg->angle_min, msg->angle_increment);

    // Get the cartesian points of the measurements
    polar2cartesian();

    // The measurements are ready and restore 
    scan_request = false;
    scan_received = true;

    return;
}


void ObjectDetection::generateAnglesVector(float a_max,float a_min,float a_inc){

    for (int i = 22; i < 666-22 ; i++)
        angle_measurements.push_back(a_min + a_inc*i); 
    return;
}

void ObjectDetection::polar2cartesian(){

    cartesian_measures.clear();

    Point cartesian_cord;
    for(int i =0; i < point_distances.size(); i++){
        cartesian_cord.x = point_distances[i]*cos(angle_measurements[i]);
        cartesian_cord.y = point_distances[i]*sin(angle_measurements[i]);
        cartesian_measures.push_back(cartesian_cord);
    }
    return;
}

void ObjectDetection::showMeas() {

    for(int k=0; k < cartesian_measures.size(); k++)
        ROS_INFO("cartesian measure (x,y): (%f , %f)", cartesian_measures[k].x, cartesian_measures[k].y);
    
}

// Abandoned: not working
std::vector<Point> ObjectDetection::HoughObjectDetection(int threshold, float angle_resolution, float distance_resolution)
{

    // first, discard lines: r = x_i*cos(theta) + y_i*sin(theta), where (x_i,y_i) is the point, (r,theta) the pair of parameters that encodes the line
    // define the accumulator: a single cell is a vector containing the coordinates of the points that define a given line
    std::vector <int> CELL;
    std::vector <std::vector <int>> angle_cells;    // a row of the accumulator
    std::vector <float> angles;
    for (float angle = 0.0; angle < 3.14159265 ; angle += angle_resolution){
        angle_cells.push_back(CELL);
        angles.push_back(angle);
    }
    std::vector <std::vector <std::vector <int>>> ACCUMULATOR;
    std::vector <float> distances;
    for (float distance = 0.0; distance < 26.0 ; distance += distance_resolution){  // 26 becase max distance is 25
        ACCUMULATOR.push_back(angle_cells);
        distances.push_back(distance);
    }

    /*
    Hence the accumulator is:
                          angle
                     0.0    . . .    \pi
                    _____________________
                0.00|___|___|___|___|___|
                .   |___|___|___|___|___|
     distance   .   |___|___|___|___|___|
                .   |___|___|___|___|___|
                26.0|___|___|___|___|___|
    */

    ROS_INFO("accumulator created");

    //for each cartesian point, compute the respective possible pairs of parameters
    for (int pt_idx = 0; pt_idx < cartesian_measures.size(); pt_idx++){
        Point point = cartesian_measures[pt_idx];

        //ROS_INFO("making point %i", pt_idx);

        // span theta angles
        for (int ang_idx = 0; ang_idx < angles.size(); ang_idx++){

            ROS_INFO("point %i: making angle %f",pt_idx, angles[ang_idx]);

            float r = abs(point.x * cos(angles[ang_idx]))+(point.y * sin(angles[ang_idx]));

            // span distances
            for (int dist_idx = 0; dist_idx < distances.size(); dist_idx++){

                while (r > distances[dist_idx])
                    dist_idx++;
                if (r < distances[dist_idx+1])
                    (ACCUMULATOR[dist_idx][ang_idx]).push_back(pt_idx);
            }
        }

    }

    ROS_INFO("accumulator filled");

    // now go through the entire accumulator to delete points corresponding to lines (make them = (0,0), then delete them)
    std::vector <int> points_to_delete;
    for (int ang_idx = 0; ang_idx < angles.size(); ang_idx++){
        for (int dist_idx = 0; dist_idx < distances.size(); dist_idx++){

            if ( (ACCUMULATOR[dist_idx][ang_idx]).size() >= threshold) {
                for (int i=0; i < (ACCUMULATOR[dist_idx][ang_idx]).size(); i++)
                    points_to_delete.push_back(ACCUMULATOR[dist_idx][ang_idx][i]);
            }
        }
    }

    ROS_INFO("points to delete found");

    Point zero;
    zero.x = 0.;
    zero.y = 0.;

    for (int i=0; i < points_to_delete.size(); i++){
        cartesian_measures[points_to_delete[i]] = zero;
    }

    std::vector<Point> points_to_return;

    for (int j=cartesian_measures.size()-1; j >=0; j--){
        //ROS_INFO("points-begin: %i",points.begin());
        if ((cartesian_measures[j].x != 0.0) && ((cartesian_measures[j].y != 0.0)))
            points_to_return.push_back(cartesian_measures[j]);
    }

    ROS_INFO("points deleted");

    return points_to_return;
}

std::vector<Point> ObjectDetection::FitCircle(){

    std::vector<Point> provisional_centers;
    obstacle_points.clear();
    // Take 3 points at a time
    for (int pt_idx1 = cartesian_measures.size()-1; pt_idx1 >= 2; pt_idx1--){

        int pt_idx2 = pt_idx1-1;
        int pt_idx3 = pt_idx1-2;

        // compute the circumference passing through the 3 points
        // x^2 + y^2 + ax + by + c = 0

        // first, square terms sqr = x^2 + y^2
        float sqr1 = (cartesian_measures[pt_idx1].x)*(cartesian_measures[pt_idx1].x) + (cartesian_measures[pt_idx1].y)*(cartesian_measures[pt_idx1].y);
        float sqr2 = (cartesian_measures[pt_idx2].x)*(cartesian_measures[pt_idx2].x) + (cartesian_measures[pt_idx2].y)*(cartesian_measures[pt_idx2].y);
        float sqr3 = (cartesian_measures[pt_idx3].x)*(cartesian_measures[pt_idx3].x) + (cartesian_measures[pt_idx3].y)*(cartesian_measures[pt_idx3].y);

        // compute a, b, c
        float b = (sqr1 - sqr3 - (sqr1 - sqr2)*(cartesian_measures[pt_idx3].x - cartesian_measures[pt_idx1].x)/(cartesian_measures[pt_idx2].x - cartesian_measures[pt_idx1].x));
        b = b / ( (cartesian_measures[pt_idx3].y - cartesian_measures[pt_idx1].y) + (cartesian_measures[pt_idx1].y - cartesian_measures[pt_idx2].y)*(cartesian_measures[pt_idx3].x - cartesian_measures[pt_idx1].x)/(cartesian_measures[pt_idx2].x - cartesian_measures[pt_idx1].x) );

        float a = (-sqr2 - b*(cartesian_measures[pt_idx2].y) + sqr1 + b*(cartesian_measures[pt_idx1].y))/(cartesian_measures[pt_idx2].x - cartesian_measures[pt_idx1].x);

        float c = -sqr1 - a*(cartesian_measures[pt_idx1].x) - b*(cartesian_measures[pt_idx1].y);

        // from a, b, c, compute radius and center
        float radius = sqrt( pow(a/2,2) + pow(b/2,2) - c);
        Point center;
        center.x = -a/2;
        center.y = -b/2;

        // if radius is too large or too small, it's not a cone (that has radius = 0.18 m)! save only centers
        if ((radius > 0.05) && (radius < 1.0)){
            provisional_centers.push_back(center);
            //ROS_INFO("radius = %f for obstacle (%f, %f)", radius, center.x, center.y);
        }
    }

    // Now, we would like to consider obstacles that are present more than n times (otherwise, it may be an outlier:
    // we expect that a cone is detected by more than just 3 points)

    for(int pt_idx = 0; pt_idx < provisional_centers.size() -1; pt_idx++){

        int subsequent_pt_idx = pt_idx +1;
        // check if subsequent center is similar (~ 0.02 m tolerance): if so, add it
        while( (abs(provisional_centers[pt_idx].x - provisional_centers[subsequent_pt_idx].x) < 0.02) && (abs(provisional_centers[pt_idx].y - provisional_centers[subsequent_pt_idx].y) < 0.02)){
            subsequent_pt_idx++;
        }

        // check how many points we considered. Consider only clusters of at least 4 subsequent equal centers, otherwise discard
        if (subsequent_pt_idx >= pt_idx + 3)
            obstacle_points.push_back(provisional_centers[pt_idx]);

        pt_idx = subsequent_pt_idx -1;
    }

    for(int i = 0; i < obstacle_points.size(); i++){
        ;
        //ROS_INFO("obstacle (%f, %f) detected",obstacle_points[i].x, obstacle_points[i].y);
    }

    return obstacle_points;
}
