#ifndef AR_PATH_NAV__PATH_GENERATOR_NODE_H_
#define AR_PATH_NAV__PATH_GENERATOR_NODE_H_

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cassert>
class PathGenerator
{
public:
  PathGenerator(ros::NodeHandle nh_);

private:
  void ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
  void get_coord();

  nav_msgs::Path calc_path();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber img_subs_;
  ros::Subscriber ar_subs_;

  ros::Publisher path_pub_;

  std::vector<ar_track_alvar_msgs::AlvarMarker> ar_marks_;
  nav_msgs::Path path_;
  nav_msgs::Path interpolated_path_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  float path_margin_;
  int interpolate_param_;
};

#endif  // AR_PATH_NAV__PATH_GENERATOR_NODE_H_
