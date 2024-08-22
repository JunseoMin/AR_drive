#include "ar_path_nav/path_generator_node.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PathGenerator::PathGenerator(ros::NodeHandle nh_)
  : nh_(nh_), priv_nh_("~"), tf_listener_(tf_buffer_), interpolate_param_(10) {
  ar_subs_ = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, &PathGenerator::ar_callback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ar_path", 10);

  close_marker_.header.frame_id = "notpublished";
}

void PathGenerator::ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  ar_marks_.clear();
  
  if (msg->markers.empty()) {
    ROS_WARN("NO MARKER return.");
    return;
  }

  ar_marks_ = msg->markers;

  check_pub();
  get_closest_marker();  // get closest marker
  set_margin();

  if (flag_pub_){
    path_.poses.clear();
    path_pub_.publish(calc_path()); // update path and publish new path
  }
  else{
    path_.header.stamp = ros::Time(0.0);
    path_pub_.publish(path_);
  }

}

void PathGenerator::get_closest_marker() {
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

  frame_id_ = "ar_marker_" + std::to_string(close_marker_.id);
  // close_marker_.header.frame_id = frame_id_;

  ROS_INFO("closest marker set");  
}

void PathGenerator::set_margin(){
  // set margined point referenced to usb cam and convert referenced to marker frame
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = close_marker_.header;
  pose_stamped.pose.position = close_marker_.pose.pose.position;

  // ar pose margin
  if(pose_stamped.pose.position.z < 0){
    pose_stamped.pose.position.z += 0.3;
  }
  else{
    pose_stamped.pose.position.z -= 0.3;
  }

  ROS_INFO("add margin!!");

  try {
    geometry_msgs::PoseStamped pose_in_marker;
    tf_buffer_.transform(pose_stamped, pose_in_marker, frame_id_, ros::Duration(1.0));  // Transform to odom frame

    pose_in_marker.pose.position.z = 0; 

    ROS_INFO("------------------------");
    ROS_INFO("converted x: %lf",pose_in_marker.pose.position.x);
    ROS_INFO("converted y: %lf",pose_in_marker.pose.position.y);
    ROS_INFO("------------------------");
    
    pose_margin_marker_ = pose_in_marker;

    error_flag_ =false;
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    error_flag_ = true;
    return;
  }
}

void PathGenerator::check_pub(){
  if(close_marker_.header.frame_id == "notpublished"){
    ROS_INFO("====================");
    ROS_INFO("initial publish set!");
    ROS_INFO("====================");
    flag_pub_ = true;
    return;
  }

  int current_marker_id = close_marker_.id;

  if (prev_marker_id_ == current_marker_id) {
    ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    ROS_INFO("Marker ID not changed: previous ID = %d, current ID = %d", prev_marker_id_, current_marker_id);
    ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    flag_pub_ = false; 
    return;
  }
  else{
    flag_pub_ = true; 
    prev_marker_id_ = current_marker_id;
    return;
  }
}

nav_msgs::Path PathGenerator::calc_path() {
  // path_ => odom referenced AR poses
  // T_ob_ odom -> base_link transform

  if(error_flag_){
    ROS_ERROR("empty path");
    ROS_WARN("tf error");
    return path_;
  }

  path_.header.frame_id = frame_id_;  // Set the frame to "odom"
  path_.header.stamp = ros::Time(0);

  set_points(); // set three points to make path
  spline_path();  // spline and publish path
  
  return path_;
}

void PathGenerator::set_points(){
  // use base_link, margined AR pose, center of AR pose and base_link
  
  // base_link point 마커 기준 base_link 좌표
  point_baselink_.pose.position.x = -close_marker_.pose.pose.position.x;  // usb cam 기준임
  point_baselink_.pose.position.y = -close_marker_.pose.pose.position.y;
  point_baselink_.pose.position.z = 0.;

  // marker point 현상태 유지
  point_margin_.pose.position.x = pose_margin_marker_.pose.position.x;
  point_margin_.pose.position.y = pose_margin_marker_.pose.position.y;
  point_margin_.pose.position.z = 0.;

  //central point
  point_central_.pose.position.x = (point_baselink_.pose.position.x * (0.95) + (1.05) * close_marker_.pose.pose.position.x)/2;
  point_central_.pose.position.y = (point_baselink_.pose.position.y * (0.95) + (1.05) * close_marker_.pose.pose.position.y)/2;
  point_central_.pose.position.z = 0;

  path_.poses.push_back(point_baselink_);
  path_.poses.push_back(point_central_);
  path_.poses.push_back(point_margin_);

  ROS_INFO("point set !!");
  std::cout << path_ << std::endl;
}

// void PathGenerator::linear_interpolate() {
//   if (path_.poses.size() < 2) {
//     ROS_ERROR("Not enough points to interpolate");
//     return;
//   }

//   std::vector<geometry_msgs::PoseStamped> interpolated_path;

//   for (size_t i = 0; i < path_.poses.size() - 1; ++i) {
//     const auto& start_pose = path_.poses[i];
//     const auto& end_pose = path_.poses[i + 1];

//     interpolated_path.push_back(start_pose);  // Start with the original point

//     for (int j = 1; j < interpolate_param_; ++j) {
//       geometry_msgs::PoseStamped interpolated_pose;
//       interpolated_pose.header = start_pose.header;

//       // Linearly interpolate the position
//       interpolated_pose.pose.position.x = start_pose.pose.position.x +
//                                           j * (end_pose.pose.position.x - start_pose.pose.position.x) / interpolate_param_;
//       interpolated_pose.pose.position.y = start_pose.pose.position.y +
//                                           j * (end_pose.pose.position.y - start_pose.pose.position.y) / interpolate_param_;
//       interpolated_pose.pose.position.z = start_pose.pose.position.z +
//                                           j * (end_pose.pose.position.z - start_pose.pose.position.z) / interpolate_param_;

//       // TODO: add orientation
//       // Interpolate orientation (if needed)

//       tf2::Quaternion q_start, q_end, q_interpolated;
//       tf2::fromMsg(start_pose.pose.orientation, q_start);
//       tf2::fromMsg(end_pose.pose.orientation, q_end);

//       q_interpolated = q_start.slerp(q_end, static_cast<double>(j) / interpolate_param_);
//       interpolated_pose.pose.orientation = tf2::toMsg(q_interpolated);

//       interpolated_path.push_back(interpolated_pose);

//     }
//   }

//   // Add the final point
//   interpolated_path.push_back(path_.poses.back());

//   // Replace the path with the interpolated path
//   path_.poses = interpolated_path;
//   ROS_INFO("point published!");
// }

void PathGenerator::spline_path() {
  if (path_.poses.size() < 2) {
    ROS_ERROR("Not enough points for spline interpolation");
    return;
  }
  std::string frame_id = "ar_marker_" + std::to_string(close_marker_.id);

  // Spline interpolation using Eigen
  Eigen::MatrixXd points(2, path_.poses.size());
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    points(0, i) = path_.poses[i].pose.position.x;
    points(1, i) = path_.poses[i].pose.position.y;
  }

  Eigen::Spline<double, 2> spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points, points.cols() - 1);

  // Interpolated path
  std::vector<geometry_msgs::PoseStamped> interpolated_path;
  for (double t = 0.0; t <= 1.0; t += 1.0 / interpolate_param_) {
    Eigen::Vector2d interpolated_point = spline(t);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = interpolated_point.x();
    pose.pose.position.y = interpolated_point.y();
    pose.pose.position.z = 0.0;

    interpolated_path.push_back(pose);
  }

  // Orientation calculation
  for (size_t i = 0; i < interpolated_path.size() - 1; ++i) {
    double dx = interpolated_path[i + 1].pose.position.x - interpolated_path[i].pose.position.x;
    double dy = interpolated_path[i + 1].pose.position.y - interpolated_path[i].pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    interpolated_path[i].pose.orientation = tf2::toMsg(q);
  }

  if (!interpolated_path.empty()) {
    interpolated_path.back().pose.orientation = interpolated_path[interpolated_path.size() - 2].pose.orientation;
  }
  path_.header.frame_id = frame_id;
  path_.header.stamp = ros::Time(0.0);
  path_.poses = interpolated_path;
  
  // std::cout << path_ << std::endl;
  ROS_INFO("Spline path generated and published!");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_generator_node");
  ros::NodeHandle nh;

  PathGenerator ar_generator(nh);

  ros::spin();

  return 0;
}
