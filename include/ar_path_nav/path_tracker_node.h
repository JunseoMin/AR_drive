#ifndef AR_PATH_NAV__PATH_TRACKER_H_
#define AR_PATH_NAV__PATH_TRACKER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <xycar_msgs/xycar_motor.h>


class PathTracker
{
public:
  PathTracker(ros::NodeHandle nh_);

private:
  int get_closest_idx();
  
  void path_callback(const nav_msgs::Path::ConstPtr& msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void timer_callback(const ros::TimerEvent& );
  void set_goal();
  void stenly();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Timer timer_;

  ros::Subscriber path_subs_;
  ros::Subscriber odom_subs_;
  ros::Publisher control_pub_;

  nav_msgs::Path path_;
  geometry_msgs::Pose goal_pose_;

  double k_;
  double velocity_;
  double curr_x_;
  double curr_y_;

  geometry_msgs::Quaternion curr_ori_;

  int goal_prev_;
};
#endif  // AR_PATH_NAV__PATH_TRACKER_H_
