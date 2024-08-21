#include "ar_path_nav/path_generator_node.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PathGenerator::PathGenerator(ros::NodeHandle nh_)
  : nh_(nh_), priv_nh_("~"), tf_listener_(tf_buffer_), interpolate_param_(10) {
  ar_subs_ = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, &PathGenerator::ar_callback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ar_path", 10);

}

void PathGenerator::ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
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
  
  double min_distance = std::numeric_limits<double>::infinity();

  for (const auto& marker : ar_marks_) {
    double dx = marker.pose.pose.position.x;
    double dy = marker.pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (min_distance > distance){
      min_distance = distance;
      close_marker_ = marker;
    }
  }  

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = close_marker_.header;
  pose_stamped.pose.position = close_marker_.pose.pose.position;

  // ar pose margin
  if(pose_stamped.pose.position.y < 0){
    pose_stamped.pose.position.y += 0.2;
  }
  else{
    pose_stamped.pose.position.y -= 0.2;
  }

  pose_stamped.header.frame_id = "usb_cam";

  try {
    geometry_msgs::PoseStamped pose_in_odom;
    tf_buffer_.transform(pose_stamped, pose_in_odom, "odom", ros::Duration(1.0));  // Transform to odom frame

    T_ob_ = tf_buffer_.lookupTransform("odom","base_link",ros::Time(0));
    pose_in_odom.pose.position.z = 0; 

    ROS_INFO("------------------------");
    ROS_INFO("converted x: %lf",pose_in_odom.pose.position.x);
    ROS_INFO("converted y: %lf",pose_in_odom.pose.position.y);
    ROS_INFO("------------------------");
    
    pose_marker_odom_ = pose_in_odom;

  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
}

nav_msgs::Path PathGenerator::calc_path() {
  // path_ => odom referenced AR poses
  // T_ob_ odom -> base_link transform
  get_coord();

  path_.header.frame_id = "odom";  // Set the frame to "odom"
  path_.header.stamp = ros::Time(0);

  if (path_.poses.empty()) {
    ROS_ERROR("empty path");
    return nav_msgs::Path();  // Return an empty path
  }

  set_points(); // set three points to make path
  linear_interpolate();
  // spline();

  path_pub_.publish(path_);
}

void PathGenerator::set_points(){
  // use base_link, margined AR pose, center of AR pose and base_link
  point_baselink_.pose.position.x = T_ob_.transform.translation.x;
  point_baselink_.pose.position.y = T_ob_.transform.translation.y;
  point_baselink_.pose.position.z = 0.;

  point_margin_.pose.position.x = pose_marker_odom_.pose.position.x;
  point_margin_.pose.position.y = pose_marker_odom_.pose.position.y;
  point_margin_.pose.position.z = 0.;

  point_central_.pose.position.x = (point_baselink_.pose.position.x + close_marker_.pose.pose.position.x)/2;
  point_central_.pose.position.y = (point_baselink_.pose.position.y + close_marker_.pose.pose.position.y)/2;
  point_central_.pose.position.z = 0;

  path_.poses.push_back(point_baselink_);
  path_.poses.push_back(point_central_);
  path_.poses.push_back(point_margin_);
}

void PathGenerator::linear_interpolate() {
  if (path_.poses.size() < 2) {
    ROS_ERROR("Not enough points to interpolate");
    return;
  }

  std::vector<geometry_msgs::PoseStamped> interpolated_path;

  for (size_t i = 0; i < path_.poses.size() - 1; ++i) {
    const auto& start_pose = path_.poses[i];
    const auto& end_pose = path_.poses[i + 1];

    interpolated_path.push_back(start_pose);  // Start with the original point

    for (int j = 1; j < interpolate_param_; ++j) {
      geometry_msgs::PoseStamped interpolated_pose;
      interpolated_pose.header = start_pose.header;

      // Linearly interpolate the position
      interpolated_pose.pose.position.x = start_pose.pose.position.x +
                                          j * (end_pose.pose.position.x - start_pose.pose.position.x) / interpolate_param_;
      interpolated_pose.pose.position.y = start_pose.pose.position.y +
                                          j * (end_pose.pose.position.y - start_pose.pose.position.y) / interpolate_param_;
      interpolated_pose.pose.position.z = start_pose.pose.position.z +
                                          j * (end_pose.pose.position.z - start_pose.pose.position.z) / interpolate_param_;

      interpolated_pose.pose.orientation.x = 0.;
      interpolated_pose.pose.orientation.y = 0.;
      interpolated_pose.pose.orientation.z = 0.;
      interpolated_pose.pose.orientation.w = 1.;

      interpolated_path.push_back(interpolated_pose);
    }
  }

  // Add the final point
  interpolated_path.push_back(path_.poses.back());

  // Replace the path with the interpolated path
  path_.poses = interpolated_path;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_generator_node");
  ros::NodeHandle nh;

  PathGenerator ar_generator(nh);

  ros::spin();

  return 0;
}
