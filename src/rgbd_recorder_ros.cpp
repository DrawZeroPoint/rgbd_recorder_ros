#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

// dynamic configure
#include <dynamic_reconfigure/server.h>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

// Xtion
float fx_ = 525.77;
float fy_ = 527.23;
float cx_ = 303.76;
float cy_ = 233.95;

double min_depth_ = 0.5;
double max_depth_ = 8;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
        sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

const string param_video_num = "/video_num";
int video_num = 0;

const string param_save_path = "/path";
string save_path = "/home/omnisky/rgbd_video/";

const string param_frame_num = "/frame_num";
int frame_num = 100; // total frame captured for each video

const string param_delay = "/delay";
int delay = 30; // ms

int idx = 0;

void depthCallback(
    const sensor_msgs::ImageConstPtr& image,
    const sensor_msgs::ImageConstPtr& imageDepth,
    const sensor_msgs::CameraInfoConstPtr& cameraInfo)
{
  if (idx > frame_num) return;

  cv_bridge::CvImagePtr imagePtr;
  if(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0)
  {
    imagePtr = cv_bridge::toCvCopy(image);
  }
  else if(image->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
          image->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
  {
    imagePtr = cv_bridge::toCvCopy(image, "mono8");
  }
  else
  {
    imagePtr = cv_bridge::toCvCopy(image, "bgr8");
  }

  cv_bridge::CvImagePtr imageDepthPtr = cv_bridge::toCvCopy(imageDepth);

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(*cameraInfo);
  fx_ = model.fx();
  fy_ = model.fy();
  cx_ = model.cx();
  cy_ = model.cy();

  bool save_images = true; // TODO: make this as a option in dynamic configure
  if (save_images) {

    if (delay > 0) cv::waitKey(delay);

    // Standard order of DAVIS dataset: /DAVIS/RGB/Categories
    string rgb_file = save_path + "/rgb/" + to_string(video_num);
    string depth_file = save_path + "/depth/" + to_string(video_num);

    if (!boost::filesystem::is_directory(rgb_file))
      boost::filesystem::create_directories(rgb_file);

    if (!boost::filesystem::is_directory(depth_file))
      boost::filesystem::create_directories(depth_file);

    std::stringstream ss;
    ss << std::setfill('0') << std::setw(5) << idx; // pad idx with 5 leading zeros

    cv::imwrite(rgb_file + ss.str() + ".png", imagePtr->image);
    cv::imwrite(depth_file + ss.str() + ".png", imageDepthPtr->image);
    idx++;
  }

  //  PointCloud::Ptr cloud (new PointCloud());
  //  gf_.getFeatures(imagePtr->image, imageDepthPtr->image, fx_, fy_, cx_, cy_, max_depth_, min_depth_, cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgbd_recorder_ros");

  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  pnh.getParam(param_save_path, save_path);
  pnh.getParam(param_delay, delay);

  // listen to the image topics
  int queueSize = 20;
  image_transport::SubscriberFilter imageSub_;
  image_transport::SubscriberFilter imageDepthSub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

  ros::NodeHandle rgb_nh(nh, "rgb");
  ros::NodeHandle depth_nh(nh, "depth");
  ros::NodeHandle rgb_pnh(pnh, "rgb");
  ros::NodeHandle depth_pnh(pnh, "depth");
  image_transport::ImageTransport rgb_it(rgb_nh);
  image_transport::ImageTransport depth_it(depth_nh);
  // !Use compressed message to speed up -- necessary!
  image_transport::TransportHints hintsRgb("compressed", ros::TransportHints(), rgb_pnh);
  image_transport::TransportHints hintsDepth("compressedDepth", ros::TransportHints(), depth_pnh);

  imageSub_.subscribe(rgb_it, rgb_nh.resolveName("/camera/rgb/image_raw"), queueSize, hintsRgb);
  imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("/camera/depth/image"), queueSize, hintsDepth);
  cameraInfoSub_.subscribe(rgb_nh, "/camera/rgb/camera_info", queueSize);

  approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>
      (MyApproxSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
  approxSyncDepth_->registerCallback(depthCallback);

  ROS_INFO("RGBD recorder initialized.\n");

  while (ros::ok()) {
    if (ros::param::has(param_video_num)) {
      int video_num_temp;
      ros::param::get(param_video_num, video_num_temp);
      if (video_num_temp > video_num && idx > frame_num) {
        // start record another video
        idx = 0;
        video_num = video_num_temp;
      }
    }
    ros::spinOnce();
  }

  return 0;
}
