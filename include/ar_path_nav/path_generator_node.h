#ifndef AR_PATH_NAV__PATH_GENERATOR_NODE_H_
#define AR_PATH_NAV__PATH_GENERATOR_NODE_H_

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class PathGenerator
{
public:
  PathGenerator(ros::NodeHandle nh_);

private:
  void image_callback(const sensor_msgs::Image::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber img_subs_;
  ros::Publisher path_pub_;

  cv::Mat curr_img_;
};

#endif  // AR_PATH_NAV__PATH_GENERATOR_NODE_H_
