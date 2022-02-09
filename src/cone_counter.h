#ifndef CONE_COUNTER_H_
#define CONE_COUNTER_H_

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <interactive_markers/interactive_marker_server.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
//Tests if any of the points are within the bounds of the box
bool testingPointforOverlap(const tf::Vector3 &min_vector, const tf::Vector3 &max_vector,
                            float x, float y, float z);

//Goes through each point in the point cloud to see if it fits the bounds of the box. Seperates each point within the box to a seperate point cloud and returns the size.
void checkPointCloud(tf::Vector3 maximum_selection, tf::Vector3 minimum_selection, pcl::PointCloud<pcl::PointXYZ> points);

// Initialses the box and it's properties.
//Marker makeBox(InteractiveMarker &msg)

// Initalised control to box, allowing it to move.
//InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg)

// Processes the position feedback when the box has moved.
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

// Initalise the arrows to move the box and their properties.
void makeBoxArrows();

//Converts the point cloud to and from a sensor message to be interpreted and published.
void pointCloudSubPub(const sensor_msgs::PointCloud2ConstPtr &input);
//Parameters
struct BoxParams {
    BoxParams():
        boxScale(0.5),
        arrowScale(1.0),
        boxRed(1.0),
        boxGreen(0.898),
        boxBlue(0.039),
        boxAlpha(0.5),
        name("BoxCounter"),
        description("Cone Counting Box"),
        headerID("Velodyne"){}
    // Scale of the measuring box
    float boxScale;
    // Size of arrow control box
    float arrowScale;
    // RGBA of box
    float boxRed;
    float boxGreen;
    float boxBlue;
    float boxAlpha;
    //Box name
    std::string name;
    // Box description
    std::string description;
    // Frame ID (velodyne)
    std::string headerID;

};

#endif
