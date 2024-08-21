#include "ar_path_nav/path_generator_node.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Dense>
#include <cmath>

class CubicSpline1D {
public:
  CubicSpline1D(const std::vector<double>& s, const std::vector<double>& y) {
    int n = s.size();
    h.resize(n - 1);
    b.resize(n - 1);
    u.resize(n - 1);
    v.resize(n - 1);
    z.resize(n);
    a.resize(n);
    c.resize(n);
    d.resize(n);

    for (int i = 0; i < n - 1; ++i) {
      h[i] = s[i + 1] - s[i];
      b[i] = (y[i + 1] - y[i]) / h[i];
    }

    v[0] = 0.0;
    u[0] = 0.0;

    for (int i = 1; i < n - 1; ++i) {
      v[i] = 2 * (h[i] + h[i - 1]);
      u[i] = 6 * (b[i] - b[i - 1]);
    }

    v[n - 1] = 0.0;
    u[n - 1] = 0.0;

    z[0] = 0.0;
    z[n - 1] = 0.0;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n - 2, n - 2);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(n - 2);

    for (int i = 0; i < n - 2; ++i) {
      A(i, i) = v[i + 1];
      if (i != 0) A(i, i - 1) = h[i];
      if (i != n - 3) A(i, i + 1) = h[i + 1];
      B(i) = u[i + 1];
    }

    Eigen::VectorXd Z = A.colPivHouseholderQr().solve(B);

    for (int i = 1; i < n - 1; ++i) {
      z[i] = Z(i - 1);
    }

    for (int i = 0; i < n - 1; ++i) {
      a[i] = y[i];
      c[i] = (z[i + 1] - z[i]) / (6 * h[i]);
      b[i] = z[i] / 2;
      d[i] = (y[i + 1] - y[i]) / h[i] - (2 * h[i] * z[i] + h[i] * z[i + 1]) / 6;
    }
  }

  double calc_position(double s) const {
    int n = h.size();
    int i = n - 1;
    for (int j = 0; j < n - 1; ++j) {
      if (s <= h[j]) {
        i = j;
        break;
      }
    }

    double ds = s - h[i];
    return a[i] + b[i] * ds + c[i] * ds * ds + d[i] * ds * ds * ds;
  }

  double calc_first_derivative(double s) const {
    int n = h.size();
    int i = n - 1;
    for (int j = 0; j < n - 1; ++j) {
        if (s <= h[j]) {
            i = j;
            break;
        }
    }
    double ds = s - h[i];
    return b[i] + 2 * c[i] * ds + 3 * d[i] * ds * ds;
  }

  double calc_second_derivative(double s) const {
    int n = h.size();
    int i = n - 1;
    for (int j = 0; j < n - 1; ++j) {
        if (s <= h[j]) {
            i = j;
            break;
        }
    }
    double ds = s - h[i];
    return 2 * c[i] + 6 * d[i] * ds;
  }

private:
  std::vector<double> h, b, u, v, z, a, c, d;
};

class CubicSpline2D {
public:
  CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y)
    : sx(calc_s(x, y), x), sy(calc_s(x, y), y) {
    s = calc_s(x, y);
  }

  std::vector<double> calc_s(const std::vector<double>& x, const std::vector<double>& y) {
    int n = x.size();
    std::vector<double> s(n, 0.0);
    for (int i = 1; i < n; ++i) {
      double dx = x[i] - x[i - 1];
      double dy = y[i] - y[i - 1];
      s[i] = s[i - 1] + std::hypot(dx, dy);
    }
    return s;
  }

  std::pair<double, double> calc_position(double s) {
    double x = sx.calc_position(s);
    double y = sy.calc_position(s);
    return {x, y};
  }

  double calc_curvature(double s) {
    double dx = sx.calc_first_derivative(s);
    double dy = sy.calc_first_derivative(s);
    double ddx = sx.calc_second_derivative(s);
    double ddy = sy.calc_second_derivative(s);
    return (ddy * dx - ddx * dy) / std::pow(dx * dx + dy * dy, 1.5);
  }

  double calc_yaw(double s) {
    double dx = sx.calc_first_derivative(s);
    double dy = sy.calc_first_derivative(s);
    return std::atan2(dy, dx);
  }

private:
  std::vector<double> s;
  CubicSpline1D sx, sy;
};

PathGenerator::PathGenerator(ros::NodeHandle nh_)
  : nh_(nh_), priv_nh_("~"), tf_listener_(tf_buffer_), path_margin_(0.2), interpolate_param_(10) {
  ar_subs_ = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, &PathGenerator::ar_callback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ar_path", 10);
}

void PathGenerator::ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  ROS_WARN("AR callback start!!");
  
  if (msg->markers.empty()) {
    ROS_WARN("NO MARKER return.");
    return;
  }

  ar_marks_ = msg->markers;
  calc_path();
}

void PathGenerator::get_coord() {
  path_.poses.clear();

  ROS_INFO("get coord start!!");
  
  for (const auto& marker : ar_marks_) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = marker.header;
        
    pose_stamped.pose.position = marker.pose.pose.position;
    pose_stamped.header.frame_id = "usb_cam";

    // Transform pose to odom frame (assuming pose_stamped is originally in usb_cam frame)
    try {
      geometry_msgs::PoseStamped pose_in_odom;
      tf_buffer_.transform(pose_stamped, pose_in_odom, "odom", ros::Duration(1.0));  // Transform to odom frame ㅅㅂ왜 이라인 안됨?

      pose_in_odom.pose.position.z = 0;
      
      ROS_INFO("------------------------");
      ROS_INFO("converted x: %lf",pose_in_odom.pose.position.x);
      ROS_INFO("converted y: %lf",pose_in_odom.pose.position.y);
      ROS_INFO("------------------------");
      
      path_.poses.push_back(pose_in_odom);

    } catch (tf2::TransformException& ex) {
      ROS_ERROR("%s // RETURN", ex.what());
      return;
    }

  }


  if (ar_marks_.size() <= 1)
  {
    ROS_WARN("one ar marker");

    path_pub_.publish(path_);
    return;
  }

  ROS_INFO("input coord set");
}

nav_msgs::Path PathGenerator::calc_path() {
  get_coord();
  ROS_INFO("calc path start!");

  path_.header.frame_id = "odom";  // Set the frame to "odom"
  path_.header.stamp = ros::Time(0);
  ROS_INFO("header_set");

  if (path_.poses.empty()) {
    ROS_ERROR("empty path");
    return nav_msgs::Path();  // Return an empty path
  }

  // Cubic interpolation
  nav_msgs::Path interpolated_path;
  interpolated_path.header = path_.header;

  std::vector<double> x, y;
  for (const auto& pose : path_.poses) {
    x.push_back(pose.pose.position.x);
    y.push_back(pose.pose.position.y);
  }

  CubicSpline2D spline(x, y);
  double ds = 0.1;

  std::vector<double> s_values;

  for (double s = 0; s <= spline.calc_s(x, y).back(); s += ds) {
    s_values.push_back(s);
  }

  for (const double s : s_values) {
    auto [ix, iy] = spline.calc_position(s);
    auto iyaw = spline.calc_yaw(s);

    geometry_msgs::PoseStamped interpolated_pose;
    interpolated_pose.header = path_.header;
    interpolated_pose.pose.position.x = ix;
    interpolated_pose.pose.position.y = iy;
    interpolated_pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, iyaw);  // Roll and pitch are set to 0, yaw is set to iyaw

    // Assign quaternion to pose orientation
    interpolated_pose.pose.orientation.x = q.x();
    interpolated_pose.pose.orientation.y = q.y();
    interpolated_pose.pose.orientation.z = q.z();
    interpolated_pose.pose.orientation.w = q.w();
        
    interpolated_path.poses.push_back(interpolated_pose);
  }

  ROS_INFO("path generated !!");

  // Publish path
  path_pub_.publish(interpolated_path);

  return interpolated_path;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "path_generator_node");
  ros::NodeHandle nh;

  PathGenerator ar_generator(nh);

  ros::spin();

  return 0;
}
