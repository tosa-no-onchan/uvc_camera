/*
* https://yhrscoding.hatenablog.com/entry/2021/09/26/104445
*/
//#include <boost/thread.hpp>

//#include <ros/ros.h>
//#include <ros/time.h>

//#include "uvc_cam/uvc_cam.h"
//#include <sensor_msgs/Image.hpp>
//#include <sensor_msgs/image_encodings.hpp>
//#include <sensor_msgs/CameraInfo.hpp>
//#include <camera_info_manager/camera_info_manager.hpp>
//#include <image_transport/image_transport.hpp>

#include "uvc_camera/single_stereocamera.hpp"


#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

//using namespace sensor_msgs;

using sensor_msgs::msg::Image;


struct control_mod {
  uint32_t id;
  int32_t val;
  std::string name;

  control_mod(uint32_t id, int32_t val, const std::string& name) {
    this->id = id;
    this->val = val;
    this->name = name;
  }
};

typedef struct control_mod control_mod_t;


/* Rotate an 8-bit, 3-channel image by 180 degrees. */
static inline void rotate(unsigned char *dst_chr, unsigned char *src_chr, int pixels) {
  struct pixel_t {
    unsigned char r, g, b;
  };

  struct pixel_t *src = (pixel_t *) src_chr;
  struct pixel_t *dst = &(((pixel_t *) dst_chr)[pixels - 1]);

  for (int i = pixels; i != 0; --i) {
    *dst = *src;
    src++;
    dst--;
  }
}

namespace uvc_camera 
{

//Single_StereoCamera::Single_StereoCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh) :
Single_StereoCamera::Single_StereoCamera(rclcpp::NodeOptions const & options)
: rclcpp::Node{"uvc_camera_stereo", options}
{
  //intra_ = options.use_intra_process_comms();
  init();
}

void Single_StereoCamera::init()
{
  /* default config values */
  width = 640;
  height = 480;
  fps = 10;
  skip_frames = 0;
  frames_to_skip = 0;
  left_device = "/dev/video0";
  right_device = "/dev/video1";
  frame = "camera";
  rotate_left = false;
  rotate_right = false;
  //left_cinfo_name="left_camera";      // for ROS
  left_cinfo_name="narrow_stereo/left";   // for ROS2
  //right_cinfo_name="right_camera";    // for ROS
  right_cinfo_name="narrow_stereo/right"; // for ROS2

  /* set up information managers */
  std::string left_url, right_url;

  bool auto_focus = false;
  int focus_absolute = 0;
  bool auto_exposure;
  int exposure_absolute;
  int brightness;
  int power_line_frequency;


  this->declare_parameter("left/camera_info_url");
  this->declare_parameter("right/camera_info_url");

  this->declare_parameter<std::string>("left_cinfo_name",left_cinfo_name);
  this->declare_parameter<std::string>("right_cinfo_name",right_cinfo_name);

  this->declare_parameter<std::string>("left/device",left_device);
  this->declare_parameter<std::string>("right/device",right_device);
  this->declare_parameter<int>("fps",fps);
  this->declare_parameter<int>("skip_frames",skip_frames);
  this->declare_parameter<bool>("left/rotate", rotate_left);
  this->declare_parameter<bool>("right/rotate", rotate_right);
  this->declare_parameter<int>("width", width);
  this->declare_parameter<int>("height", height);
  this->declare_parameter<std::string>("frame_id", frame);


  this->declare_parameter<bool>("auto_focus", auto_focus);
  this->declare_parameter<int>("focus_absolute", focus_absolute);

  this->declare_parameter("auto_exposure");
  this->declare_parameter("exposure_absolute");
  this->declare_parameter("brightness");
  declare_parameter("power_line_frequency");


  //pnode.getParam("left/camera_info_url", left_url);
  // left/camera_info_url
  //get_parameter<std::string>("left/camera_info_url", left_url);
  get_parameter<std::string>("left/camera_info_url", left_url);
  //RCLCPP_INFO(get_logger(), "left/camera_info_url: %s", left_url.c_str());
  std::cout << "left/camera_info_url:"<< left_url.c_str() << std::endl;

  //pnode.getParam("right/camera_info_url", right_url);
  get_parameter<std::string>("right/camera_info_url", right_url);
  //RCLCPP_INFO(get_logger(), "right/camera_info_url: %s", right_url.c_str());
  std::cout << "right/camera_info_url:"<< right_url.c_str() << std::endl;


  /* pull other configuration */
  //pnode.getParam("left/device", left_device);
  get_parameter<std::string>("left/device", left_device);
  //RCLCPP_INFO(get_logger(), "left/device: %s", left_url.c_str());
  std::cout << "left/device:"<< left_device.c_str() << std::endl;

  //pnode.getParam("right/device", right_device);
  get_parameter<std::string>("right/device", right_device);
  std::cout << "right/device:"<< right_device.c_str() << std::endl;

  //pnode.getParam("fps", fps);
  get_parameter<int>("fps", fps);
  std::cout << "fps:"<< fps << std::endl;

  //pnode.getParam("skip_frames", skip_frames);
  get_parameter<int>("skip_frames", skip_frames);
  std::cout << "skip_frames:"<< fps << std::endl;

  //pnode.getParam("left/rotate", rotate_left);
  get_parameter<bool>("left/rotate", rotate_left);
  if(rotate_left)
    std::cout << "left/rotate: true" << std::endl;
  else
    std::cout << "left/rotate: flase" << std::endl;

  //pnode.getParam("right/rotate", rotate_right);
  get_parameter<bool>("right/rotate", rotate_right);
  if(rotate_right)
    std::cout << "right/rotate: true" << std::endl;
  else
    std::cout << "right/rotate: flase" << std::endl;

  //pnode.getParam("width", width);
  get_parameter<int>("width", width);
  std::cout << "width:"<< width << std::endl;
  //pnode.getParam("height", height);
  get_parameter<int>("height", height);
  std::cout << "height:"<< height << std::endl;

  //pnode.getParam("frame_id", frame);
  get_parameter<std::string>("frame_id", frame);
  std::cout << "frame_id:"<< frame.c_str() << std::endl;


  //left_info_mgr.loadCameraInfo(left_url);
  //right_info_mgr.loadCameraInfo(right_url);
  //------------------
  // CameraInfoManager( rclcpp::Node * node, const std::string & cname = "camera", const std::string & url = "");
  //-----------------
  // cname= は、 left/camera_info_url: example-left.yaml の camera_name を使用
  // camera_name: left_camera
  left_info_mgr = std::make_shared<camera_info_manager::CameraInfoManager>(this, left_cinfo_name,left_url);
  //left_info_mgr->loadCameraInfo(left_url);

  // cname= は、 right/camera_info_url: example-right.yaml の camera_name を使用
  right_info_mgr = std::make_shared<camera_info_manager::CameraInfoManager>(this, right_cinfo_name,right_url);
  //right_info_mgr->loadCameraInfo(right_url);

  //if (options.use_intra_process_comms()) {
  if(intra_){

    // https://yhrscoding.hatenablog.com/entry/2021/09/26/104445
    rclcpp::QoS video_qos(1);
    video_qos.reliable();
    //video_qos.best_effort();
    video_qos.durability_volatile();

    /* advertise image streams and info streams */
    //left_pub = it.advertise("left/image_raw", 1);
    //left_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 1);
    left_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw", video_qos);

    //right_pub = it.advertise("right/image_raw", 1);
    //right_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 1);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", video_qos);

    //left_info_pub = node.advertise<CameraInfo>("left/camera_info", 1);
    //left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 1);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", video_qos);

    //right_info_pub = node.advertise<CameraInfo>("right/camera_info", 1);
    //right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 1);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", video_qos);

  }
  else{
    //#define USE_TEST_VAL
    //#ifdef USE_TEST_VAL
      rmw_qos_profile_t video_qos_profile = rmw_qos_profile_sensor_data;
      video_qos_profile.depth=1;

      //video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;

      //video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
      video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
      //video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      
      left_camera_transport_pub_ = image_transport::create_camera_publisher(this, "left/image_raw",video_qos_profile);
      right_camera_transport_pub_ = image_transport::create_camera_publisher(this, "right/image_raw",video_qos_profile);
    //#else
    //  left_camera_transport_pub_ = image_transport::create_camera_publisher(this, "left/image_raw");
    //  right_camera_transport_pub_ = image_transport::create_camera_publisher(this, "right/image_raw");
    //#endif
  }


  /* initialize the cameras */
  cam_left =
      new uvc_cam::Cam(left_device.c_str(), uvc_cam::Cam::MODE_RGB,
		       width*2, height, fps);
  cam_left->set_motion_thresholds(100, -1);

  //cam_right =
  //    new uvc_cam::Cam(right_device.c_str(), uvc_cam::Cam::MODE_RGB,
	//	       width, height, fps);
  //cam_right->set_motion_thresholds(100, -1);


  //if (pnode.getParam("auto_focus", auto_focus)) {
  if (get_parameter("auto_focus", auto_focus)) {
    cam_left->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
    //cam_right->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
  }

  //if (pnode.getParam("focus_absolute", focus_absolute)) {
  if (get_parameter("focus_absolute", focus_absolute)) {
    cam_left->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
    //cam_right->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
  }

  //if (pnode.getParam("auto_exposure", auto_exposure)) {
  if (get_parameter("auto_exposure", auto_exposure)) {
   int val;
    if (auto_exposure) {
      val = V4L2_EXPOSURE_AUTO;
    } else {
      val = V4L2_EXPOSURE_MANUAL;
    }
    cam_left->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
    //cam_right->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
  }

  //if (pnode.getParam("exposure_absolute", exposure_absolute)) {
  if (get_parameter("exposure_absolute", exposure_absolute)) {
    cam_left->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
    //cam_right->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
  }

  //if (pnode.getParam("brightness", brightness)) {
  if (get_parameter("brightness", brightness)) {
    cam_left->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
    //cam_right->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
  }

  //if (pnode.getParam("power_line_frequency", power_line_frequency)) {
  if (get_parameter("power_line_frequency", power_line_frequency)) {
    int val;
    if (power_line_frequency == 0) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
    } else if (power_line_frequency == 50) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
    } else if (power_line_frequency == 60) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
    } else {
      printf("power_line_frequency=%d not supported. Using auto.\n", power_line_frequency);
      val = V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
    }
    cam_left->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
    //cam_right->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
  }



  // TODO:
  // - add params for
  //   contrast
  //   saturation
  //   hue
  //   white balance temperature, auto and manual
  //   gamma
  //   sharpness
  //   backlight compensation
  //   exposure auto priority
  //   zoom
  // - add generic parameter list:
  //   [(id0, val0, name0), (id1, val1, name1), ...]

  std::cout << "Single_StereoCamera::init(): complete!!" << std::endl;


  /* and turn on the streamer */
  ok = true;
  image_thread = boost::thread(boost::bind(&Single_StereoCamera::feedImages, this));
}

void Single_StereoCamera::sendInfo(rclcpp::Time time, std::shared_ptr<sensor_msgs::msg::Image> const & img_l, 
      std::shared_ptr<sensor_msgs::msg::Image> const & img_r) {

  //CameraInfoPtr info_left(new CameraInfo(left_info_mgr.getCameraInfo()));
  std::shared_ptr<sensor_msgs::msg::CameraInfo> info_left = std::make_shared<sensor_msgs::msg::CameraInfo>(left_info_mgr->getCameraInfo());
  //printf("%s",info_left);

  if (!checkCameraInfo(*img_l, *info_left)) {
    *info_left = sensor_msgs::msg::CameraInfo{};
    info_left->height = img_l->height;
    info_left->width = img_l->width;
  }

  //CameraInfoPtr info_right(new CameraInfo(right_info_mgr.getCameraInfo()));
  std::shared_ptr<sensor_msgs::msg::CameraInfo> info_right = std::make_shared<sensor_msgs::msg::CameraInfo>(right_info_mgr->getCameraInfo());

  if (!checkCameraInfo(*img_r, *info_right)) {
    *info_right = sensor_msgs::msg::CameraInfo{};
    info_right->height = img_r->height;
    info_right->width = img_r->width;
  }

  info_left->header.stamp = time;
  info_right->header.stamp = time;
  info_left->header.frame_id = frame;
  info_right->header.frame_id = frame;

  if(intra_){
    //left_pub.publish(image_left);
    left_pub_->publish(*img_l);

    //right_pub.publish(image_right);
    right_pub_->publish(*img_r);

    //left_info_pub.publish(info_left);
    left_info_pub_->publish(*info_left);
    //right_info_pub.publish(info_right);
    right_info_pub_->publish(*info_right);
  }
  else{
    left_camera_transport_pub_.publish(img_l, info_left);
    right_camera_transport_pub_.publish(img_r, info_right);
  }
}


void Single_StereoCamera::feedImages() {
  unsigned int pair_id = 0;
  while (ok) {
    unsigned char *frame_left = NULL;
    //unsigned char *frame_right = NULL;
    uint32_t bytes_used_left;
    //uint32_t bytes_used_right;

    //ros::Time capture_time = ros::Time::now();
    rclcpp::Time capture_time = now();

    // test
    //printf("%s=",capture_time);


    int left_idx = cam_left->grab(&frame_left, bytes_used_left);
    //int right_idx = cam_right->grab(&frame_right, bytes_used_right);

    /* Read in every frame the camera generates, but only send each
     * (skip_frames + 1)th frame. This reduces effective frame
     * rate, processing time and network usage while keeping the
     * images synchronized within the true framerate.
     */
    if (skip_frames == 0 || frames_to_skip == 0) {
      //if (frame_left && frame_right) {
      if (frame_left) {
        //ImagePtr image_left(new Image);
        auto image_left = std::make_shared<sensor_msgs::msg::Image>();

        //ImagePtr image_right(new Image);
        auto image_right = std::make_shared<sensor_msgs::msg::Image>();

        image_left->height = height;
        image_left->width = width;
        image_left->step = 3 * width;
        //image_left->encoding = image_encodings::RGB8;
        image_left->encoding = sensor_msgs::image_encodings::RGB8;
        image_left->header.stamp = capture_time;
        //image_left->header.seq = pair_id;

        image_right->height = height;
        image_right->width = width;
        image_right->step = 3 * width;
        //image_right->encoding = image_encodings::RGB8;
        image_right->encoding = sensor_msgs::image_encodings::RGB8;
        image_right->header.stamp = capture_time;
        //image_right->header.seq = pair_id;

        image_left->header.frame_id = frame;
        image_right->header.frame_id = frame;

        image_left->data.resize(image_left->step * image_left->height);
        image_right->data.resize(image_right->step * image_right->height);

        //if (rotate_left){
          // not support
          //rotate(&image_left->data[0], frame_left, width * height);
        //}
        //else{
          // 左右横に並んでいます。
          //memcpy(&image_left->data[0], frame_left, width * height * 3);
          copy_frame(&image_left->data[0],&image_right->data[0],frame_left,width,height);
        //}
        //if (rotate_right){
          // not support
          //rotate(&image_right->data[0], frame_right, width * height);
        //}
        //else{
          //memcpy(&image_right->data[0], frame_right, width * height * 3);
        //}

        //left_pub.publish(image_left);
        //right_pub.publish(image_right);
        sendInfo(capture_time,image_left,image_right);

        ++pair_id;
      }
      frames_to_skip = skip_frames;
    } 
    else {
      frames_to_skip--;
    }

    if (frame_left)
      cam_left->release(left_idx);
    //if (frame_right)
    //  cam_right->release(right_idx);
  }
}

void Single_StereoCamera::copy_frame(unsigned char *left,unsigned char *right,unsigned char *frame,int width,  int height) {
  int step = width * 3;
  for(int i=0;i<height;i++){
    memcpy(left, frame, step);
    frame += step;
    left += step;
    memcpy(right, frame, step);
    frame += step;
    right += step;
  }
}

Single_StereoCamera::~Single_StereoCamera() {
  ok = false;
  image_thread.join();
  if (cam_left)
    delete cam_left;
  //if (cam_right)
  //  delete cam_right;
}

bool Single_StereoCamera::checkCameraInfo(
  sensor_msgs::msg::Image const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}


}

//RCLCPP_COMPONENTS_REGISTER_NODE(uvc_camera::Single_StereoCamera)
