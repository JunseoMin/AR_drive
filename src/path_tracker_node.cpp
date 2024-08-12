#include "ar_path_nav/path_tracker_node.h"

PathTracker::PathTracker(ros::NodeHandle nh_)
:nh_(nh_),priv_nh_("~")
{
  path_subs_ = nh_.subscribe("/ar_path",10,&PathTracker::path_callback,this);

}

void PathTracker::path_callback(nav_msgs::Path::ConstPtr& msg){
}

int main(int argc, char **argv){
  ros::init(argc, argv, "path_tracker_node");
  ros::NodeHandle nh;

  PathTracker pathtrack(nh);

  ros::spin();

  return 0;
}