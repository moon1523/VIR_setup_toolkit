#ifndef FUNCTIONS_HH_
#define FUNCTIONS_HH_

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// #include <vtkPoints.h>
// #include <vtkPolyData.h>
// #include <vtkUnsignedCharArray.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <limits>

#ifdef K4A_FOUND
#include <k4a/k4a.hpp>
#endif

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/eigen.hpp>

#include <sl/Camera.hpp>

using namespace std;
using namespace Eigen;


#ifdef K4A_FOUND
#define VERIFY(result, error)                                                                            \
if(result != K4A_RESULT_SUCCEEDED)                                                                   \
{                                                                                                    \
    printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
    exit(1);                                                                                         \
}
k4a_device_configuration_t get_default_config();
cv::Mat color_to_opencv(const k4a_image_t im);
cv::Mat depth_to_opencv(const k4a_image_t im);
void WritePointCloud(const k4a_image_t point_image, 
                     const k4a_image_t color_image,
                     string fileName);
void WriteTrasnformedPointCloudToRefBoard(const k4a_image_t point_image,
                                          const k4a_image_t color_image,
										  string fileName);
#endif


void WriteTrasnformedPointCloudToRefBoard(sl::Camera& zed,
										  const sl::Mat slMat,
										  string serial_number);
void Write_Fixed_Camera_Transformation_Matrix(string serial_number);

Eigen::Vector4d weighted_averaging_quaternions(
  const std::vector<Eigen::Vector4d>& quaternions,
  const std::vector<double>& weights);
Eigen::Vector4d averaging_quaternions(const std::vector<Eigen::Vector4d> &quaternions);
Eigen::Vector3d averaging_translations(const std::vector<Eigen::Vector3d> &translations);
double averaging_weights(const std::vector<double> &weights);
void cv2eigen_Affine3d(const cv::Affine3d &c, Eigen::Affine3d &e);
Eigen::Affine3d averaging_Affine3d(const vector<Eigen::Affine3d> &affs);


class Timer
{
public:
    Timer() : start_(0), time_(0) {}

    void start() {
    	start_ = cv::getTickCount();
    }
    void stop() {
        CV_Assert(start_ != 0);
        int64 end = cv::getTickCount();
        time_ += end - start_;
        start_ = 0;
    }

    double time() {
        double ret = time_ / cv::getTickFrequency();
        time_ = 0;
        return ret;
    }

private:
    int64 start_, time_;
};

#endif
