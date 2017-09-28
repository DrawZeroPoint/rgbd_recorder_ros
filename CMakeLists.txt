cmake_minimum_required(VERSION 2.8.3)
project(rgbd_recorder_ros)

# You only need add this to use C++ 11 features
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Boost COMPONENTS system filesystem regex REQUIRED)

find_package(OpenCV REQUIRED)
message("Found OpenCV ${OpenCV_VERSION}")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} "src/rgbd_recorder_ros.cpp")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
