/*
* single_stereocamera.h
*/

#ifndef SINGLE_STEREOCAMERA_HPP_
#define SINGLE_STEREOCAMERA_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <boost/thread.hpp>

//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

//#include <ros/time.h>

#include "uvc_cam/uvc_cam.h"
//#include <sensor_msgs/Image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
//#include <sensor_msgs/CameraInfo.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <image_transport/image_transport.hpp>

//using namespace std::chrono_literals;


namespace uvc_camera {

class Single_StereoCamera : public rclcpp::Node
{
  public:
    Single_StereoCamera(): Node("uvc_camera_stereo"){
      init();
    }

    Single_StereoCamera(const std::string & node_name): Node(node_name){
      init();
    }

    Single_StereoCamera(const std::string & node_name,rclcpp::NodeOptions const & options): Node(node_name, options){
      init();
    }

    void init();

    void onInit();
    void sendInfo(rclcpp::Time time);
    void feedImages();
    void copy_frame(unsigned char *left,unsigned char *right,unsigned char *frame,int width,  int height);
    ~Single_StereoCamera();

  private:
    //ros::NodeHandle node, pnode;
    //image_transport::ImageTransport it;
    image_transport::CameraPublisher camera_transport_pub_;
    bool ok;

    uvc_cam::Cam *cam_left, *cam_right;
    int width, height, fps, skip_frames, frames_to_skip;
    std::string left_device, right_device, frame;
    bool rotate_left, rotate_right;

    std::shared_ptr<camera_info_manager::CameraInfoManager> left_info_mgr, right_info_mgr;

    //image_transport::Publisher left_pub, right_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;

    //ros::Publisher left_info_pub, right_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

    size_t count_;

    boost::thread image_thread;
    //std::thread image_thread_;

};

} // namespace uvc_camera

#endif  // SINGLE_STEREOCAMERA_H_
