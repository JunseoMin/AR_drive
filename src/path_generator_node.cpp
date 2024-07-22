#include "ar_path_nav/path_generator_node.h"


PathGenerator::PathGenerator(ros::NodeHandle nh_)
: nh_(nh_), priv_nh_("~")
{
  img_subs_ = nh_.subscribe<sensor_msgs::Image>("/usb_cam/image_raw",10,&PathGenerator::image_callback,this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ar_path",1000);

}

void PathGenerator::image_callback(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  nav_msgs::Path ar_path;
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    curr_img_ = cv_ptr->image;
    cv::imshow("current img",curr_img_);
    cv::waitKey(1);

    PathGenerator::get_coord();
    ar_path = PathGenerator::calc_path();

    path_pub_.publish(ar_path);
    ROS_INFO("PATH PUBLISHED!");
  }
  catch(const std::exception& e)
  {
    ROS_WARN("PATH NOT PUBLISHED! ERROR :");
    std::cerr << e.what() << '\n';
  }
}

void PathGenerator::get_coord(){
  //get AR marker's coordinate
}

nav_msgs::Path PathGenerator::calc_path(){
  // calc path to follow and return
}

int main(int argc, char **argv){
  ros::init(argc, argv, "path_generator_node");
  ros::NodeHandle nh;

  PathGenerator ar_generator(nh);

  ros::spin();

  return 0;
}