#include "ar_path_nav/path_generator_node.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Dense>
#include <cmath>

#include <ar_path_nav/circular.hpp>

PathGenerator::PathGenerator(ros::NodeHandle nh_)
  : nh_(nh_), priv_nh_("~"), tf_listener_(tf_buffer_), path_margin_(0.2), interpolate_param_(10) {
  ar_subs_ = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, &PathGenerator::ar_callback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ar_path", 10);
}

void PathGenerator::ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  ROS_WARN("AR callback start!!");
  
  if (msg->markers.empty()) {
    ROS_WARN("NO MARKER return.");
    return;
  }

  ar_marks_ = msg->markers;
  calc_path();
}

void PathGenerator::get_coord() {
  path_.poses.clear();

  ROS_INFO("get coord start!!");
  
  for (const auto& marker : ar_marks_) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = marker.header;
        
    pose_stamped.pose.position = marker.pose.pose.position;
    pose_stamped.header.frame_id = "usb_cam";

    // Transform pose to odom frame (assuming pose_stamped is originally in usb_cam frame)
    try {
      geometry_msgs::PoseStamped pose_in_odom;
      tf_buffer_.transform(pose_stamped, pose_in_odom, "odom", ros::Duration(1.0));  // Transform to odom frame

      pose_in_odom.pose.position.z = 0;
      
      ROS_INFO("------------------------");
      ROS_INFO("converted x: %lf",pose_in_odom.pose.position.x);
      ROS_INFO("converted y: %lf",pose_in_odom.pose.position.y);
      ROS_INFO("------------------------");
      
      path_.poses.push_back(pose_in_odom);

    } catch (tf2::TransformException& ex) {
      ROS_ERROR("%s // RETURN", ex.what());
      return;
    }

  }


  if (ar_marks_.size() <= 1)
  {
    ROS_WARN("one ar marker");

    path_pub_.publish(path_);
    return;
  }

  ROS_INFO("input coord set");
}

nav_msgs::Path PathGenerator::calc_path() {
  get_coord();
  ROS_INFO("calc path start!");

  path_.header.frame_id = "odom";  // Set the frame to "odom"
  path_.header.stamp = ros::Time(0);

  ROS_INFO("header_set");

  if (path_.poses.empty()) {
    ROS_ERROR("empty path");
    return nav_msgs::Path();  // Return an empty path
  }
  
  ROS_INFO("path generated !!");

  // Publish path
  path_pub_.publish(interpolated_path);

  return interpolated_path;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "path_generator_node");
  ros::NodeHandle nh;

  PathGenerator ar_generator(nh);

  ros::spin();

  return 0;
}
