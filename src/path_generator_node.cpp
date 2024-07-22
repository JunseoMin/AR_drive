#include "ar_path_nav/path_generator_node.h"


PathGenerator::PathGenerator(ros::NodeHandle nh_)
: nh_(nh_), priv_nh_("~")
{
  img_subs_ = nh_.subscribe<sensor_msgs::Image>("/image_raw",10,&PathGenerator::image_callback,this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ar_path",1000);

}

void PathGenerator::image_callback(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}

int main(int argc, char **argv){
  ros::init(argc, argv, "path_generator_node");
  ros::NodeHandle nh;

  PathGenerator ar_generator(nh);

  ros::spin();

  return 0;
}