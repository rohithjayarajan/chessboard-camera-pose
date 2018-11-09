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
 *  @file    CameraPose.cpp
 *  @author  rohithjayarajan
 *  @date 11/8/2018
 *  @version 1.1
 *
 *  @brief Camera Pose Class Definition
 *
 *  @section DESCRIPTION
 *
 *  Class definition for camera pose computations
 *
 */

#include "CameraPose.hpp"

CameraPose::CameraPose() {
  chessboardEdge = 0.023;
  patternSize_ = cvSize(9, 6);
}

CameraPose::CameraPose(double chessboardEdge_) {
  chessboardEdge = chessboardEdge_;
  patternSize_ = cvSize(9, 6);
}

CameraPose::~CameraPose() {}

void CameraPose::drawAxes(cv::Mat &src_, cv::Mat &dst_,
                          std::vector<cv::Point2d> &imgPts_,
                          std::vector<cv::Point2f> &cornersSP_) {
  src_.copyTo(dst_);
  cv::arrowedLine(dst_, cornersSP_[0], imgPts_[0], cv::Scalar(0, 0, 255), 2,
                  cv::LINE_AA, 0);
  cv::arrowedLine(dst_, cornersSP_[0], imgPts_[1], cv::Scalar(0, 255, 0), 2,
                  cv::LINE_AA, 0);
  cv::arrowedLine(dst_, cornersSP_[0], imgPts_[2], cv::Scalar(255, 0, 0), 2,
                  cv::LINE_AA, 0);
}

void CameraPose::helper_RT_ImgPoints(cv::Mat &src_, cv::Mat &rvec_,
                                     cv::Mat &tvec_,
                                     std::vector<cv::Point2f> &corners_,
                                     std::vector<cv::Point3d> &boardPts_) {
  cv::TermCriteria termcrit(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                            30, 0.001);
  cv::cornerSubPix(src_, corners_, cv::Size(11, 11), cv::Size(-1, -1),
                   termcrit);
  cv::solvePnPRansac(boardPts_, corners_, this->cameraMatrix_,
                     this->distCoeffs_, rvec_, tvec_, false, 100, 8.0, 0.99,
                     cv::noArray(), cv::SOLVEPNP_EPNP);
}

bool CameraPose::poseEstimationImage(std::string frame_) {
  cv::Mat frame, gray, rvec, tvec, outputFrame;
  std::vector<cv::Point2f> corners;
  std::vector<cv::Point2d> imgPts;
  std::vector<cv::Point3d> boardPts, axis;
  for (int i = 0; i < patternSize_.height; i++) {
    for (int j = 0; j < patternSize_.width; j++) {
      boardPts.push_back(
          cv::Point3d(i * chessboardEdge, j * chessboardEdge, 0));
    }
  }
  axis.push_back(cv::Point3d(3 * chessboardEdge, 0.0, 0.0));
  axis.push_back(cv::Point3d(0.0, 3 * chessboardEdge, 0.0));
  axis.push_back(cv::Point3d(0.0, 0.0, -3 * chessboardEdge));
  frame = cv::imread(frame_);
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  cv::findChessboardCorners(
      gray, this->patternSize_, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
  helper_RT_ImgPoints(gray, rvec, tvec, corners, boardPts);
  cv::projectPoints(axis, rvec, tvec, this->cameraMatrix_, this->distCoeffs_,
                    imgPts, cv::noArray(), 0);
  drawAxes(frame, outputFrame, imgPts, corners);
  std::cout << "rotation " << std::endl;
  std::cout << "x: " << rvec.at<double>(0, 0) * (180 / M_PI) << " degree"
            << std::endl
            << "y: " << rvec.at<double>(1, 0) * (180 / M_PI) << " degree"
            << std::endl
            << "z: " << rvec.at<double>(2, 0) * (180 / M_PI) << " degree"
            << std::endl;
  std::cout << "translation " << std::endl;
  std::cout << "x: " << tvec.at<double>(0, 0) << " meter" << std::endl
            << "y: " << tvec.at<double>(1, 0) << " meter" << std::endl
            << "z: " << tvec.at<double>(2, 0) << " meter" << std::endl;
  cv::imshow("poseOP", outputFrame);
  cv::waitKey(0);
  return true;
}

bool CameraPose::poseEstimationCamera() {
  // open the default camera
  cv::VideoCapture cap(2);
  // check if we succeeded
  if (!cap.isOpened()) {
    return false;
  }
  while (true) {
    cv::Mat frame, gray, rvec, tvec, outputFrame;
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point2d> imgPts;
    std::vector<cv::Point3d> boardPts, axis;
    for (int i = 0; i < patternSize_.height; i++) {
      for (int j = 0; j < patternSize_.width; j++) {
        boardPts.push_back(
            cv::Point3d(i * chessboardEdge, j * chessboardEdge, 0));
      }
    }
    axis.push_back(cv::Point3d(3 * chessboardEdge, 0.0, 0.0));
    axis.push_back(cv::Point3d(0.0, 3 * chessboardEdge, 0.0));
    axis.push_back(cv::Point3d(0.0, 0.0, -3 * chessboardEdge));
    // get a new frame from camera
    cap >> frame;
    cv::imshow("inputFrame", frame);
    cv::waitKey(0);
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::findChessboardCorners(
        gray, this->patternSize_, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
    helper_RT_ImgPoints(gray, rvec, tvec, corners, boardPts);
    cv::projectPoints(axis, rvec, tvec, this->cameraMatrix_, this->distCoeffs_,
                      imgPts, cv::noArray(), 0);
    drawAxes(frame, outputFrame, imgPts, corners);
    std::cout << "rotation " << std::endl;
    std::cout << "x: " << rvec.at<double>(0, 0) * (180 / M_PI) << " degree"
              << std::endl
              << "y: " << rvec.at<double>(1, 0) * (180 / M_PI) << " degree"
              << std::endl
              << "z: " << rvec.at<double>(2, 0) * (180 / M_PI) << " degree"
              << std::endl;
    std::cout << "translation " << std::endl;
    std::cout << "x: " << tvec.at<double>(0, 0) << " meter" << std::endl
              << "y: " << tvec.at<double>(1, 0) << " meter" << std::endl
              << "z: " << tvec.at<double>(2, 0) << " meter" << std::endl;
    cv::imshow("poseOP", outputFrame);
    if (cv::waitKey(30) >= 0) {
      break;
    }
  }
  return true;
}

void CameraPose::setPatternSize(int checkerboardRows, int checkerboardCols) {
  patternSize_ = cvSize(checkerboardRows, checkerboardCols);
}
