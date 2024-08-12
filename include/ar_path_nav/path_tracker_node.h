#ifndef AR_PATH_NAV__PATH_TRACKER_H_
#define AR_PATH_NAV__PATH_TRACKER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include <xycar_msgs/xycar_motor.h>

class PathTracker
{
public:
  PathTracker(ros::NodeHandle nh_);

private:
  int get_closest_idx();
  
  void path_callback(nav_msgs::Path::ConstPtr& msg);
  void timer_callback();
  void set_goal();
  void stenly();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber path_subs_;
  ros::Publisher control_pub;

};
#endif  // AR_PATH_NAV__PATH_TRACKER_H_
