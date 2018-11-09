/******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * /**
 *  Copyright 2018 rohithjayarajan
 *  @file    CameraPose.hpp
 *  @author  rohithjayarajan
 *  @date 11/8/2018
 *  @version 1.1
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
  cv::Size_<int> patternSize_;
  double chessboardEdge;

 public:
  CameraPose();
  CameraPose(double chessboardEdge_);
  ~CameraPose();
  void drawAxes(cv::Mat &src_, cv::Mat &dst_, std::vector<cv::Point2d> &imgPts_,
                std::vector<cv::Point2f> &cornersSP_);
  void helper_RT_ImgPoints(cv::Mat &src_, cv::Mat &rvec_, cv::Mat &tvec_,
                           std::vector<cv::Point2f> &corners_,
                           std::vector<cv::Point3d> &boardPts_);
  bool poseEstimationImage(std::string frame_);
  bool poseEstimationCamera();
  void setPatternSize(int checkerboardRows, int checkerboardCols);
};

#endif  // INCLUDE_CAMERAPOSE_HPP_
