#include "ar_path_nav/path_generator_node.h"

PathGenerator::PathGenerator(ros::NodeHandle nh_)
: nh_(nh_), priv_nh_("~")
{
  ar_subs_ = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker",10,&PathGenerator::ar_callback,this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ar_path",1000);
}

void PathGenerator::ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
  if (msg->markers.empty()){
    return;
  }

  ar_marks_ = msg->markers;
  calc_path();
  control();
}

void PathGenerator::get_coord(){
  path_.poses.clear();
  for (const auto& marker : ar_marks_) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = marker.header;
    pose_stamped.pose = marker.pose.pose;
    path_.poses.push_back(pose_stamped);
  }
  ROS_INFO("input coord set");
}

nav_msgs::Path PathGenerator::calc_path(){
  assert(!ar_marks_.empty());
  get_coord();

  path_.header.frame_id = "base_link";
  path_.header.stamp = ros::Time::now();

  // 경로의 위치 보정
  if(path_.poses.front().pose.position.y > 0){
    for (auto& pose : path_.poses){
      pose.pose.position.y -= path_margin_;
    }
  } else {
    for (auto& pose : path_.poses){
      pose.pose.position.y += path_margin_;
    }
  }

  ROS_INFO("path generated !!");

  // publish path
  path_pub_.publish(path_);

  return path_;
}

void PathGenerator::control(){
  //stenly control
  
}

int main(int argc, char **argv){
  ros::init(argc, argv, "path_generator_node");
  ros::NodeHandle nh;

  PathGenerator ar_generator(nh);

  ros::spin();

  return 0;
}
