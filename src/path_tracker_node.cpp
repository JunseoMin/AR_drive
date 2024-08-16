#include "ar_path_nav/path_tracker_node.h"

PathTracker::PathTracker(ros::NodeHandle nh_)
:nh_(nh_),priv_nh_("~")
{
  path_subs_ = nh_.subscribe("/ar_path", 10, &PathTracker::path_callback ,this);
  control_pub_ = nh_.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 10);

  ros::Timer timer = nh_.createTimer(ros::Duration(0.1), &PathTracker::timer_callback, this);
}

void PathTracker::path_callback(const nav_msgs::Path::ConstPtr& msg){
  path_.header = msg->header;
  path_.poses = msg->poses;
}

void PathTracker::timer_callback(const ros::TimerEvent&)
{
  set_goal();
  stenly();
}

void PathTracker::set_goal()
{
  int closest_idx = get_closest_idx();
  int goal_idx = closest_idx;

  if (goal_idx >= path_.poses.size())
  {
    goal_idx = path_.poses.size() - 1;
  }

  goal_pose_ = path_.poses[goal_idx].pose;
}

int PathTracker::get_closest_idx()
{
  double min_dist = std::numeric_limits<double>::max();
  int closest_idx = 0;

  for (size_t i = 0; i < path_.poses.size(); ++i)
  {
    double dx = path_.poses[i].pose.position.x;
    double dy = path_.poses[i].pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist){
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

void PathTracker::stenly()
{
  double dx = goal_pose_.position.x;
  double dy = goal_pose_.position.y;

  double heading_to_goal = std::atan2(dy, dx);
  double current_yaw = 0.0;

  double heading_error = heading_to_goal - current_yaw;
  double cross_track_error = std::sqrt(dx * dx + dy * dy);
  if(heading_error < 0){
    cross_track_error = -cross_track_error;
  }
  double control_steering = heading_error + std::atan2(k_ * cross_track_error, velocity_);

  xycar_msgs::xycar_motor motor_msg;
  motor_msg.angle = control_steering;
  motor_msg.speed = velocity_;

  control_pub_.publish(motor_msg);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "path_tracker_node");
  ros::NodeHandle nh;

  PathTracker pathtrack(nh);

  ros::spin();

  return 0;
}