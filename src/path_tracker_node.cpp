#include "ar_path_nav/path_tracker_node.h"
#include <tf/transform_datatypes.h>  // To use for quaternion to yaw conversion

PathTracker::PathTracker(ros::NodeHandle nh_)
: nh_(nh_), priv_nh_("~"), velocity_(1.0), k_(1.0)
{
  path_subs_ = nh_.subscribe("/ar_path", 10, &PathTracker::path_callback, this);
  control_pub_ = nh_.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 10);

  timer_ = nh_.createTimer(ros::Duration(0.1), &PathTracker::timer_callback, this);

  odom_subs_ = nh_.subscribe("/odom", 10, &PathTracker::odom_callback, this);
}

void PathTracker::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  curr_x_ = msg->pose.pose.position.x;
  curr_y_ = msg->pose.pose.position.y;
  curr_ori_ = msg->pose.pose.orientation;
}

void PathTracker::path_callback(const nav_msgs::Path::ConstPtr& msg) {
  path_.header = msg->header;
  path_.poses = msg->poses;
}

void PathTracker::timer_callback(const ros::TimerEvent&) {
  if (path_.poses.empty()) {
    ROS_WARN("Received empty path, skipping this cycle.");
    return;
  }
  ROS_INFO("timer callback start!");
  set_goal();
  stenly();
}

void PathTracker::set_goal() {
  int closest_idx = get_closest_idx();
  int goal_idx = closest_idx;

  if (goal_idx >= path_.poses.size()) {
    goal_idx = path_.poses.size() - 1;
  }

  if (goal_idx < goal_prev_ ){
    goal_idx = goal_prev_ + 1;
  }

  goal_prev_ = goal_idx;

  goal_pose_ = path_.poses[goal_idx].pose;
  ROS_INFO("goal pose set!! idx: %d", goal_idx);
}

int PathTracker::get_closest_idx() {
  double min_dist = std::numeric_limits<double>::max();
  int closest_idx = 0;
  ROS_INFO("current x: %f current y: %f", curr_x_, curr_y_);

  for (size_t i = 0; i < path_.poses.size(); ++i) {
    double dx = curr_x_ - path_.poses[i].pose.position.x;
    double dy = curr_y_ - path_.poses[i].pose.position.y;

    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

void PathTracker::stenly() {
  double dx = goal_pose_.position.x - curr_x_;
  double dy = goal_pose_.position.y - curr_y_;

  double heading_to_goal = std::atan2(dy, dx);

  tf::Quaternion q(
      curr_ori_.x,
      curr_ori_.y,
      curr_ori_.z,
      curr_ori_.w);
  tf::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double heading_error = heading_to_goal - yaw;
  double cross_track_error = std::sqrt(dx * dx + dy * dy);
  
  if (heading_error < 0) {
    cross_track_error = -cross_track_error;
  }

  double control_steering = heading_error + std::atan2(k_ * cross_track_error, velocity_);

  xycar_msgs::xycar_motor motor_msg;
  motor_msg.angle = control_steering;
  motor_msg.speed = velocity_;

  ROS_INFO("motor published (angle): %lf", control_steering);
  control_pub_.publish(motor_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_tracker_node");
  ros::NodeHandle nh;

  PathTracker pathtrack(nh);

  ros::spin();

  return 0;
}
