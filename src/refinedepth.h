#ifndef REFINEDEPTH_H
#define REFINEDEPTH_H

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <iostream>
#include <random>
#include <vector>

#include "global.h"

using namespace std;
using namespace cv;

class RefineDepth
{
public:
  RefineDepth(string proto_folder, int gpu_id);
  void refineDepth(Mat rgb_in, Mat depth_in, Mat &depth_out);

protected:
  boost::shared_ptr<caffe::Net<float> > net_;

private:
  void forward(vector<float> depth_in, vector<vector<float> > features_in,
               vector<vector<float> > features_out, Mat &depth_out);
  void setParams();
  void wrapInputLayer(vector<float> depth_in, vector<vector<float> > features_in,
                      vector<vector<float> > features_out);
  void getOutput(cv::Mat &depth_out);
};

#endif // REFINEDEPTH_H
