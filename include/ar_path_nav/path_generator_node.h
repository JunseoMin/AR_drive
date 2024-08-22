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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ar_path_nav/circular.hpp>
#include <ar_path_nav/spline.hpp>

#include <cassert>

// #include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>

class PathGenerator
{
public:
  PathGenerator(ros::NodeHandle nh_);

private:
  void ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
  void get_coord();
  void set_points();
  void linear_interpolate();
  void spline_path();

  nav_msgs::Path calc_path();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber img_subs_;
  ros::Subscriber ar_subs_;

  ros::Publisher path_pub_;

  std::vector<ar_track_alvar_msgs::AlvarMarker> ar_marks_;
  nav_msgs::Path path_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ar_track_alvar_msgs::AlvarMarker close_marker_;

  geometry_msgs::PoseStamped pose_marker_odom_;
  geometry_msgs::TransformStamped T_ob_;

  geometry_msgs::PoseStamped point_margin_;
  geometry_msgs::PoseStamped point_central_;
  geometry_msgs::PoseStamped point_baselink_;

  int interpolate_param_;

  bool error_flag_;
};

#endif  // AR_PATH_NAV__PATH_GENERATOR_NODE_H_
