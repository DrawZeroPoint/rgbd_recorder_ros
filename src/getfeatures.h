#ifndef GETFEATURES_H
#define GETFEATURES_H

//STL
#include <iostream>
#include <math.h>
#include <vector>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class GetFeatures
{
public:
  GetFeatures();

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  void getFeatures(cv::Mat color, cv::Mat depth, float fx, float fy, float cx, float cy, float maxDepth, float minDepth,
                   PointCloud::Ptr  &cloudSource);

private:
  template<class T>
  inline bool uIsFinite(const T & value)
  {
    return std::isfinite(value);
  }

  PointCloud::Ptr cloudFromDepthRGB(const cv::Mat &imageRgb, const cv::Mat &imageDepth,
                                    float cx, float cy, float fx, float fy, float maxDepth, float minDepth);
  float getDepth(const cv::Mat &depthImage, float x, float y, bool smoothing, float maxZError, bool estWithNeighborsIfNull);
  pcl::PointXYZ projectDepthTo3D(const cv::Mat &depthImage, float x, float y, float cx, float cy, float fx, float fy,
                                 bool smoothing, float maxZError);
};

#endif // GETFEATURES_H
