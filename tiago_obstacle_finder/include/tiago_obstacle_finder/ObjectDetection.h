/**
 * @file ObjectDetection.h
 * @brief Implementation of tools and methods to perform the detection of
 * cylindrical obstacles, using laser scan informations.
 * @date 2021-12-19
 * 
 * 
 */


// C++ Packages
#include <math.h>
#include <vector>

// ROS Packages
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


struct Point{
    float x;
    float y;
};


/**
 * @brief Class used to perform object detection using the
 * laser scan data output and Hough transform for reject straight 
 * lines measurements (i.e. walls). 
 * 
 */    
class ObjectDetection
{
public:
    ObjectDetection();

    ~ObjectDetection() {};

    /**
     * @brief Hough transform: from cartesian coordinates returns 
     * obstacles positions (circular obstacles only)
     * 
     * @param threshold threshold for accumulator cells to discard lines
     * @param angle_resolution 
     * @param distance_resolution 
     * @return std::vector<Point>  centers of the obstacles
     */
    // Abandoned: not working
    std::vector<Point> HoughObjectDetection(int threshold, float angle_resolution, float distance_resolution);

    /**
     * @brief Determine if some points are arranged as circles
     * @return Centers of the found circles
     */
    std::vector<Point> FitCircle();

    /**
     * @brief Determine if the callback reads and stores the scan data
     * 
     */
    void scanRequest() { scan_request = true; scan_received = false; }

     /**
     * @brief If we have received measures from scan 
     * 
     * @return true 
     * @return false 
     */
    bool scanReceived() {return scan_received; };

    /**
     * @brief Read the laser scan topic and get the measurements
     * 
     * @param msg 
     */
    void LaserCallback(const sensor_msgs::LaserScanConstPtr& msg);

    /**
     * @brief Show scan measurements in cartesian points
     * 
     */
    void showMeas();

     /**
     * @brief input: state represent ad integer; output: state description
     * 
     */
    std::string fromEnumToFeedback(int state);


private:
    
    /**
     * @brief Generates a vector where the inner elements are angles 
     * associated to laser measurements based on the scan msg params.
     * 
     * @param angle_max 
     * @param angle_min 
     * @param angle_increment 
     */
    void generateAnglesVector(float a_max,float a_min,float a_inc);

    /**
     * @brief Transforms the polar measurements in cartesian coords
     * 
     */
    void polar2cartesian();

    
    //// Vector of point distances (polar distances)
    std::vector<float> point_distances;

    //// Vector of polar measures
    std::vector<float> angle_measurements;

    //// Vector of measurements expressed in cartesian points
    std::vector<Point> cartesian_measures;

    //// Vector of obstacles expressed in cartesian points
    std::vector<Point> obstacle_points;

    //// Determine if we want to read the scan topic
    bool scan_request;

    //// Determine if we have received scan measures
    bool scan_received;
};


