cmake_minimum_required(VERSION 3.5)
project(uvc_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system)
#find_package(V4L2 REQUIRED)
set(V4L2_LIBRARIES v4l2)

# add by nishi
add_executable(uvc_single_stereo_node src/single_stereo_node.cpp  src/single_stereo.cpp  src/uvc_cam.cpp)

target_include_directories(uvc_single_stereo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#target_link_libraries(uvc_single_stereo_node ${Boost_LIBRARIES} ${V4L2_LIBRARIES} ${catkin_LIBRARIES})

ament_target_dependencies(uvc_single_stereo_node
  "rclcpp"
  "rclcpp_components"
  "image_transport"
  "camera_info_manager")

target_link_libraries(uvc_single_stereo_node ${Boost_LIBRARIES} ${V4L2_LIBRARIES} ${catkin_LIBRARIES})

#target_compile_options(uvc_single_stereo_node PRIVATE -Werror)
target_compile_options(uvc_single_stereo_node PRIVATE -Wunused-variable)

install(
  TARGETS 
  uvc_single_stereo_node
  DESTINATION lib/${PROJECT_NAME}

  #EXPORT export_${PROJECT_NAME}
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  #RUNTIME DESTINATION bin
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
