/*
* single_stereo_node.cpp
*/
//#include <ros/ros.h>
//#include "rclcpp/rclcpp.hpp"
//#include <nodelet/loader.h>

#include "uvc_camera/single_stereo.hpp"

using namespace std::chrono_literals;

int main (int argc, char **argv) {
  rclcpp::init(argc, argv);

  //auto node = rclcpp::Node::make_shared("uvc_camera_stereo");
  //auto node = std::make_shared<uvc_camera::Single_StereoCamera>("uvc_camera_stereo");
  //auto node = std::make_shared<uvc_camera::Single_StereoCamera>("uvc_camera_stereo",rclcpp::NodeOptions{});

  //uvc_camera::Single_StereoCamera stereo(ros::NodeHandle(), ros::NodeHandle("~"));

  //rclcpp::spin(node);

  //rclcpp::spin(std::make_shared<uvc_camera::Single_StereoCamera>("uvc_camera_stereo"));
  rclcpp::spin(std::make_shared<single_stereo::Single_StereoCamera>(rclcpp::NodeOptions{}));


  rclcpp::shutdown();
  //node = nullptr;
  return 0;
}

