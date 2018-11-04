/**
 *  Copyright 2018 rohithjayarajan
 *  @file    CameraPose.hpp
 *  @author  rohithjayarajan
 *  @date 11/1/2018
 *  @version 1.0
 *
 *  @brief Camera Pose Class Declaration
 *
 *  @section DESCRIPTION
 *
 *  Class for camera pose computations
 *
 */

#ifndef INCLUDE_CAMERAPOSE_HPP_
#define INCLUDE_CAMERAPOSE_HPP_
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

class CameraPose {
 private:
  cv::Mat cameraMatrix_ = (cv::Mat_<float>(3, 3) << 482.4307, 0, 188.9788, 0,
                           488.7866, 151.3949, 0, 0, 1);
  std::vector<double> distCoeffs_ = {-0.3264, 0.3535, -0.0082, 0.0029, -0.4346};
  cv::Size_<int> patternSize_ = cvSize(9, 6);

 public:
  CameraPose();
  ~CameraPose();
  void drawAxes(cv::Mat &src_, cv::Mat &dst_, std::vector<cv::Point2d> &imgPts_,
                std::vector<cv::Point2f> &cornersSP_);
  void helper_RT_ImgPoints(cv::Mat &src_, cv::Mat &rvec_, cv::Mat &tvec_,
                           std::vector<cv::Point2f> &corners_,
                           std::vector<cv::Point3d> &boardPts_);
  void poseEstimation(std::string frame_);
};

#endif  // INCLUDE_CAMERAPOSE_HPP_
