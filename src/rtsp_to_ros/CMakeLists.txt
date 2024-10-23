cmake_minimum_required(VERSION 3.0.2)
project(rtsp_to_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)


catkin_package(
  CATKIN_DEPENDS roscpp image_transport cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS} 

)

add_executable(rtsp_to_ros src/rtsp_to_ros.cpp)

target_link_libraries(rtsp_to_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_INCLUDE_DIRS} 

)
