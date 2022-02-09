
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
#include "cone_counter.h"

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
BoxParams params;
using namespace visualization_msgs;

bool testingPointforOverlap(const tf::Vector3 &min_vector, const tf::Vector3 &max_vector,
                            float x, float y, float z)
{
  bool overlap = true;
  overlap = (min_vector.getX() > x || max_vector.getX() < x) ? false : overlap;
  overlap = (min_vector.getZ() > z || max_vector.getZ() < z) ? false : overlap;
  overlap = (min_vector.getY() > y || max_vector.getY() < y) ? false : overlap;
  return overlap;
}

void checkPointCloud(tf::Vector3 maximum_selection, tf::Vector3 minimum_selection, pcl::PointCloud<pcl::PointXYZ> points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_in(new pcl::PointCloud<pcl::PointXYZ>);
  for (unsigned i = 0; i < points.size(); i++)
  {
    if (testingPointforOverlap(minimum_selection, maximum_selection, points[i].x, points[i].y, points[i].z))
    {
      points_in->push_back(points[i]);
    }
  }
  std::string num_points = std::to_string(points_in->size());

  ROS_INFO_STREAM("The number of points inside the box: " << num_points);
}

Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = params.boxScale;
  marker.scale.y = params.boxScale;
  marker.scale.z = params.boxScale;
  marker.color.r = params.boxRed;
  marker.color.g = params.boxGreen;
  marker.color.b = params.boxBlue;
  marker.color.a = params.boxAlpha;

  return marker;
}

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  tf::Vector3 max_selection(feedback->pose.position.x + (params.boxScale / 2), feedback->pose.position.y + (params.boxScale / 2), feedback->pose.position.z + (params.boxScale / 2));
  tf::Vector3 min_selection(feedback->pose.position.x - (params.boxScale / 2), feedback->pose.position.y - (params.boxScale / 2), feedback->pose.position.z - (params.boxScale / 2));
  // ROS_INFO_STREAM(" Max selection is now at "
  //                 << max_selection.getX() << ", " << max_selection.getY()
  //                 << ", " << max_selection.getZ());
  // ROS_INFO_STREAM(" Min selection is now at "
  //                 << min_selection.getX() << ", " << min_selection.getY()
  //                 << ", " << min_selection.getZ());
  checkPointCloud(max_selection, min_selection, *cloud);
}

void makeBoxArrows()
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = params.headerID;
  int_marker.scale = 1;

  int_marker.name = params.name;
  int_marker.description = params.description;
  InteractiveMarkerControl control;
  control.always_visible = true;

  tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}

void pointCloudSubPub(const sensor_msgs::PointCloud2ConstPtr &input)
{
  sensor_msgs::PointCloud2 output;

  // Convert from ROS message to PCL point cloud
  pcl::fromROSMsg(*input, *cloud);
  pcl::toROSMsg(*cloud, output);

  output.header.stamp = ros::Time::now();
  output.header.frame_id = params.headerID;
  pub.publish(output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;

  n.param("/cone_counter/boxScale", params.boxScale, params.boxScale);
  n.param("/cone_counter/arrowScale", params.arrowScale, params.arrowScale);
  n.param("/cone_counter/boxRed", params.boxRed, params.boxRed);
  n.param("/cone_counter/boxGreen", params.boxGreen, params.boxGreen);
  n.param("/cone_counter/boxBlue", params.boxBlue, params.boxBlue);
  n.param("/cone_counter/boxAlpha", params.boxAlpha, params.boxAlpha);
  n.param("/cone_counter/name", params.name, params.name);
  n.param("/cone_counter/description", params.description, params.description);
  n.param("/cone_counter/headerID", params.headerID, params.headerID);

  ros::Subscriber sub = n.subscribe("velodyne_points", 1, pointCloudSubPub);

  pub = n.advertise<sensor_msgs::PointCloud2>("/cone_counting_output", 1);

  server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));
  makeBoxArrows();
  server->applyChanges();

  ros::spin();

  server.reset();
}
