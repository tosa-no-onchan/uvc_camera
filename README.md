# uvc_camera_single    
Single USB Stereo Camera (HBV-1780-2 S2.0) uvc_camera for ros2が    
使える様に、プログラムを修正しました。   
    
    開発環境    
    ubuntu mate 20.04    
    ros2: foxy    
    camera: HBV-1780-2 S2.0    
    
ビルド方法。    
1. ros2 版 uvc_camera を git clone します。    
$ cd ~/colcon_ws/src    
$ git https://github.com/tosa-no-onchan/uvc_camera.git    
$ cd ../    
$ colcon build --packages-select uvc_camera    
    
    
実行    
    2. run direct    
    $ ros2 run uvc_camera uvc_single_stereo_node    
    
    3. run lunch    
    $ ros2 launch /home/your-id/colcon_ws/src/uvc_camera/launch/single_stereo_node.launch.xml or .py    
    
    
製造元へのお願い!!    
    
    About stereo frame layout of HBV-1780-2 S2.0.    
    If possible, arrange the captioned image frames vertically rather than horizontally.    
    Then, programing for sepatete each left frame and right frame will become more easy.    
    
キャリブレーション    
    
キャリブレーションは、[ROS rtabmap_ros 自作 Stereo Camera ](http://www.netosa.com/blog/2021/09/ros-rtabmap-ros-stereo-camera.html)    
    
