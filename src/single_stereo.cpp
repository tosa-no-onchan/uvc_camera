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

#include "uvc_camera/single_stereo.hpp"


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

//namespace uvc_camera 
namespace single_stereo
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
  trace_ = false;
	int qos = 0;
  intra_ = false;   // true -> old type / false -> New type
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
  std::string left_url="file:///home/nishi/colcon_ws/src/uvc_camera/example-left.yaml";
  std::string right_url="file:///home/nishi/colcon_ws/src/uvc_camera/example-right.yaml";

  bool auto_focus = false;
  int focus_absolute = 0;

  // set defaults value by nishi 2023.2.13
  int auto_exposure=-1;		// 0 to 3
  int exposure_absolute;
  int brightness=-100;			// -64 to 64
  int power_line_frequency=-1;	// 0 / 50 /60 / -1 

  trace_ = this->declare_parameter<bool>("trace", trace_);
  intra_ = this->declare_parameter<bool>("intra", intra_);
	qos = this->declare_parameter<int>("qos", qos);

  left_url = this->declare_parameter<std::string>("left/camera_info_url",left_url);
  right_url = this->declare_parameter<std::string>("right/camera_info_url",right_url);

  left_cinfo_name = this->declare_parameter<std::string>("left_cinfo_name",left_cinfo_name);
  right_cinfo_name = this->declare_parameter<std::string>("right_cinfo_name",right_cinfo_name);

  left_device = this->declare_parameter<std::string>("left/device",left_device);
  right_device = this->declare_parameter<std::string>("right/device",right_device);
  fps = this->declare_parameter<int>("fps",fps);
  skip_frames = this->declare_parameter<int>("skip_frames",skip_frames);
  rotate_left = this->declare_parameter<bool>("left/rotate", rotate_left);
  rotate_right = this->declare_parameter<bool>("right/rotate", rotate_right);
  width = this->declare_parameter<int>("width", width);
  height = this->declare_parameter<int>("height", height);
  frame = this->declare_parameter<std::string>("frame_id", frame);


  auto_focus = this->declare_parameter<bool>("auto_focus", auto_focus);
  focus_absolute = this->declare_parameter<int>("focus_absolute", focus_absolute);

  auto_exposure = this->declare_parameter<int>("auto_exposure",auto_exposure);
  exposure_absolute = this->declare_parameter<int>("exposure_absolute",exposure_absolute);
  brightness = this->declare_parameter<int>("brightness",brightness);
  power_line_frequency = declare_parameter<int>("power_line_frequency",power_line_frequency);


  if(intra_){
    RCLCPP_INFO(this->get_logger(),"intra: True");
  }
  else{
    RCLCPP_INFO(this->get_logger(),"intra: False");
  }

  if(trace_){
    RCLCPP_INFO(this->get_logger(),"trace: True");
  }
  else{
    RCLCPP_INFO(this->get_logger(),"trace: False");
  }

  RCLCPP_INFO(this->get_logger(),"qos: %d",qos);

  //pnode.getParam("left/camera_info_url", left_url);
  // left/camera_info_url
  RCLCPP_INFO(get_logger(), "left/camera_info_url: %s", left_url.c_str());

  //pnode.getParam("right/camera_info_url", right_url);
  // right/camera_info_url
  RCLCPP_INFO(get_logger(), "right/camera_info_url: %s", right_url.c_str());

  /* pull other configuration */
  //pnode.getParam("left/device", left_device);
  RCLCPP_INFO(get_logger(), "left/device: %s", left_device.c_str());

  //pnode.getParam("right/device", right_device);
  RCLCPP_INFO(get_logger(), "right/device: %s", right_device.c_str());

  //pnode.getParam("fps", fps);
  RCLCPP_INFO(get_logger(), "fps: %d", fps);

  //pnode.getParam("skip_frames", skip_frames);
  RCLCPP_INFO(get_logger(), "skip_frames: %d", fps);
  //std::cout << "skip_frames:"<< skip_frames << std::endl;

  //pnode.getParam("left/rotate", rotate_left);
  if(rotate_left)
    std::cout << "left/rotate: true" << std::endl;
  else
    std::cout << "left/rotate: flase" << std::endl;

  //pnode.getParam("right/rotate", rotate_right);
  if(rotate_right)
    std::cout << "right/rotate: true" << std::endl;
  else
    std::cout << "right/rotate: flase" << std::endl;

  //pnode.getParam("width", width);
  RCLCPP_INFO(get_logger(), "width: %d", width);
  //std::cout << "width:"<< width << std::endl;

  //pnode.getParam("height", height);
  RCLCPP_INFO(get_logger(), "height: %d", height);
  //std::cout << "height:"<< height << std::endl;

  //pnode.getParam("frame_id", frame);
  RCLCPP_INFO(get_logger(), "frame_id: %s", frame.c_str());
  //std::cout << "frame_id:"<< frame.c_str() << std::endl;


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
    video_qos.reliability((rmw_qos_reliability_policy_t)qos);
    //video_qos.durability_volatile();

    /* advertise image streams and info streams */
    //left_pub = it.advertise("left/image_raw", 1);
    left_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw", video_qos);

    //right_pub = it.advertise("right/image_raw", 1);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", video_qos);

    //left_info_pub = node.advertise<CameraInfo>("left/camera_info", 1);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", video_qos);

    //right_info_pub = node.advertise<CameraInfo>("right/camera_info", 1);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", video_qos);

  }
  else{
    //#define USE_TEST_VAL
    //#ifdef USE_TEST_VAL

      rmw_qos_profile_t video_qos_profile = rmw_qos_profile_sensor_data;
      video_qos_profile.depth=1;
      switch(qos){
        case 0:
          video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
        break;
        case 1:
          video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        break;
        case 2:
          video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        break;
      }
      //video_qos_profile.reliability = (rmw_qos_reliability_policy_t)qos;

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
  if (auto_focus) {
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
  if (brightness >= -64 && brightness <= 64) {
    cam_left->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
    //cam_right->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
  }

  //if (pnode.getParam("power_line_frequency", power_line_frequency)) {
  if (power_line_frequency >=0 ) {
    int val;
    if (power_line_frequency == 0) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
    } 
    else if (power_line_frequency == 50) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
    } 
    else if (power_line_frequency == 60) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
    } 
    else {
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

  // 監視ルーチンを入れてみる。 2023.3.11 
  // from usb_camera_driver.cpp
  timer_ = this->create_wall_timer(700ms, std::bind(&Single_StereoCamera::TimerCallback, this));
  //timer_ = this->create_wall_timer(300ms, std::bind(&Single_StereoCamera::TimerCallback, this));

  /* and turn on the streamer */
  ok = true;
  //image_thread = boost::thread(boost::bind(&Single_StereoCamera::feedImages, this));
  image_thread = std::thread(std::bind(&Single_StereoCamera::feedImages, this));

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

  trace_sts_=20;

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
    // http://docs.ros.org/en/jade/api/image_transport/html/classimage__transport_1_1CameraPublisher.html
    // https://github.com/ros-perception/image_transport_tutorials
    left_camera_transport_pub_.publish(img_l, info_left);

    trace_sts_=23;

    right_camera_transport_pub_.publish(img_r, info_right);
  }
}


void Single_StereoCamera::feedImages() {
  //unsigned int pair_id_ = 0;

  double sec_dur = 3.0;
  //uint32_t nsec_dur = 1000000000 * 2; // 2[sec]

  rclcpp::Time start_t = now();
  rclcpp::WallRate rate(fps);

  int cnt=0;

  while (ok) {
    if(fps < 15){
      rate.sleep();
    }
    unsigned char *frame_left = NULL;
    //unsigned char *frame_right = NULL;
    uint32_t bytes_used_left;
    //uint32_t bytes_used_right;

    //ros::Time capture_time = ros::Time::now();
    // https://docs.ros2.org/beta3/api/rclcpp/classrclcpp_1_1Time.html
    rclcpp::Time capture_time = now();

    // test
    //printf("%s=",capture_time);
    trace_sts_=1;

    int left_idx = cam_left->grab(&frame_left, bytes_used_left);
    //int right_idx = cam_right->grab(&frame_right, bytes_used_right);
    if(left_idx >= 0){
      /* Read in every frame the camera generates, but only send each
      * (skip_frames + 1)th frame. This reduces effective frame
      * rate, processing time and network usage while keeping the
      * images synchronized within the true framerate.
      */

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

        trace_sts_=2;

        //left_pub.publish(image_left);
        //right_pub.publish(image_right);
        sendInfo(capture_time,image_left,image_right);

        trace_sts_= 3;

        ++pair_id_;

        //if(trace_ && (pair_id_ % 60)==0){
        //  //RCLCPP_INFO(get_logger(), "pair_id: %d", pair_id_);
        //  std::cout << "Single_StereoCamera::feedImages(): #2 pair_id_: " << pair_id_ << std::endl;
        //}
      }
      frames_to_skip = skip_frames;

      if (frame_left)
        cam_left->release(left_idx);
      //if (frame_right)
      //  cam_right->release(right_idx);

      cnt++;
      rclcpp::Time cur_t = now();
      rclcpp::Duration elapsed = cur_t - start_t;
      // 2[sec] 経過
      if(elapsed.seconds() >= sec_dur){
        std::cout << "Single_StereoCamera::feedImages(): #2 rate: " << cnt/3 << std::endl;

        start_t = now();
        cnt=0;
      }

    }
    else{
      std::cout << "Single_StereoCamera::feedImages(): #3 grab error!!" << std::endl;
    }
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

void Single_StereoCamera::TimerCallback(){
  if(pair_id_ == pair_id_prev_){

    //RCLCPP_INFO(get_logger(), "TimerCallback() trace_sts_: %d", trace_sts_);
    std::cout << "TimerCallback() trace_sts_:" << trace_sts_ << std::endl;
  }
  pair_id_prev_=pair_id_;

  //std::cout << "TimerCallback() #2 trace_sts_:" << trace_sts_ << std::endl;

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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.


RCLCPP_COMPONENTS_REGISTER_NODE(single_stereo::Single_StereoCamera)
