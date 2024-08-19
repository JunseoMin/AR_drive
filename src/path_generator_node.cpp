#include "ar_path_nav/path_generator_node.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PathGenerator::PathGenerator(ros::NodeHandle nh_)
: nh_(nh_), priv_nh_("~"), tf_listener_(tf_buffer_), path_margin_(0.2), interpolate_param_(10)
{
  ar_subs_ = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker",10,&PathGenerator::ar_callback,this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ar_path",10);
}

void PathGenerator::ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
  if (msg->markers.empty()){
    return;
  }

  ar_marks_ = msg->markers;
  calc_path();
}

void PathGenerator::get_coord(){
  path_.poses.clear();
  for (const auto& marker : ar_marks_) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = marker.header;

    pose_stamped.pose.position.x = -marker.pose.pose.position.x;
    pose_stamped.pose.position.y = marker.pose.pose.position.y;
    pose_stamped.pose.position.z = marker.pose.pose.position.z;

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(marker.pose.pose.orientation, q_orig);

    q_rot.setRPY(0, 0, M_PI);

    q_new = q_rot * q_orig;
    q_new.normalize();

    tf2::convert(q_new, pose_stamped.pose.orientation);

    // Transform pose to odom frame (assuming pose_stamped is originally in base_link frame)
    try {
      pose_stamped.header.frame_id = "base_link";  // Original frame
      tf_buffer_.transform(pose_stamped, pose_stamped, "odom", ros::Duration(1.0));  // Transform to odom frame
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      continue;
    }

    path_.poses.push_back(pose_stamped);
  }

  ROS_INFO("input coord set");
}

nav_msgs::Path PathGenerator::calc_path(){
  assert(!ar_marks_.empty());
  get_coord();

  path_.header.frame_id = "odom";  // Set the frame to "odom"
  path_.header.stamp = ros::Time::now();

  if(path_.poses.front().pose.position.y > 0){
    for (auto& pose : path_.poses){
      pose.pose.position.y -= path_margin_;
    }
  } else {
    for (auto& pose : path_.poses){
      pose.pose.position.y += path_margin_;
    }
  }

  // Linear interpolate path
  nav_msgs::Path interpolated_path;
  interpolated_path.header = path_.header;

  for (size_t i = 0; i < path_.poses.size() - 1; ++i) {
    const auto& p1 = path_.poses[i];
    const auto& p2 = path_.poses[i + 1];
    
    interpolated_path.poses.push_back(p1);

    for (int j = 1; j <= interpolate_param_; ++j) {
      geometry_msgs::PoseStamped interpolated_pose;
      interpolated_pose.header = p1.header;

      double t = static_cast<double>(j) / (interpolate_param_ + 1);

      interpolated_pose.pose.position.x = p1.pose.position.x + t * (p2.pose.position.x - p1.pose.position.x);
      interpolated_pose.pose.position.y = p1.pose.position.y + t * (p2.pose.position.y - p1.pose.position.y);
      interpolated_pose.pose.position.z = 0;

      tf2::Quaternion q1, q2;
      tf2::convert(p1.pose.orientation, q1);
      tf2::convert(p2.pose.orientation, q2);
      
      tf2::Quaternion q_interpolated = q1.slerp(q2, t);
      tf2::convert(q_interpolated, interpolated_pose.pose.orientation);
      
      interpolated_path.poses.push_back(interpolated_pose);
    }
  }
  
  interpolated_path.poses.push_back(path_.poses.back());  

  ROS_INFO("path generated !!");

  // publish path
  path_pub_.publish(interpolated_path);

  return path_;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "path_generator_node");
  ros::NodeHandle nh;

  PathGenerator ar_generator(nh);

  ros::spin();

  return 0;
}
