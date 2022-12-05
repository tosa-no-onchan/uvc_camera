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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

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

    Single_StereoCamera(rclcpp::NodeOptions const & options);

    void init();

    void onInit();
    //void sendInfo(rclcpp::Time time);
    void sendInfo(rclcpp::Time time, std::shared_ptr<sensor_msgs::msg::Image> const & img_l, 
                    std::shared_ptr<sensor_msgs::msg::Image> const & img_r);
    void feedImages();
    void copy_frame(unsigned char *left,unsigned char *right,unsigned char *frame,int width,  int height);
    ~Single_StereoCamera();

  private:
    //ros::NodeHandle node, pnode;
    //image_transport::ImageTransport it;
    image_transport::CameraPublisher camera_transport_pub_;
    bool ok;
    bool intra_= false;
    
    uvc_cam::Cam *cam_left, *cam_right;
    int width, height, fps, skip_frames, frames_to_skip;
    std::string left_device, right_device, frame;
    std::string left_cinfo_name,right_cinfo_name;
    bool rotate_left, rotate_right;

    std::shared_ptr<camera_info_manager::CameraInfoManager> left_info_mgr, right_info_mgr;

    // Publisher used for intra process comm
    //image_transport::Publisher left_pub, right_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;

    //ros::Publisher left_info_pub, right_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

    // Publisher used for inter process comm
    image_transport::CameraPublisher left_camera_transport_pub_;
    image_transport::CameraPublisher right_camera_transport_pub_;

    //std::shared_ptr<camera_info_manager::CameraInfoManager> left_cinfo_;
    //std::shared_ptr<camera_info_manager::CameraInfoManager> right_cinfo_;


    size_t count_;

    boost::thread image_thread;
    //std::thread image_thread_;

    bool checkCameraInfo(
      sensor_msgs::msg::Image const & img,
      sensor_msgs::msg::CameraInfo const & ci);

};

} // namespace uvc_camera

#endif  // SINGLE_STEREOCAMERA_H_
