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

#include <iostream>
#include "CameraPose.hpp"

int main(int argc, char *argv[]) {
  std::cout
      << "[Image Format expected: ./app/pose-app "
         "../frames/frame_<number>.png <size_of_chessboard_edge_in_meter>]"
      << std::endl
      << "[If camera used, Format expected: ./app/pose-app "
         "<size_of_chessboard_edge_in_meter>]"
      << std::endl;
  if (argc == 3) {
    CameraPose poseImage(atof(argv[2]));
    std::cout << "Starting pose estimation for: " << argv[1] << std::endl;
    char defaultConfig;
    std::cout << "Do you want to use default chessboard inner rows(=9) and "
                 "inner columns(=6) configuration? Press y(es)/n(o)"
              << std::endl;
    std::cin >> defaultConfig;
    if (defaultConfig == 'n') {
      int checkerboardRows_, checkerboardCols_;
      std::cout << "Enter number of chessboard inner rows: ";
      std::cin >> checkerboardRows_;
      std::cout << std::endl;
      std::cout << "Enter number of chessboard inner columns: ";
      std::cin >> checkerboardCols_;
      std::cout << std::endl;
      poseImage.setPatternSize(checkerboardRows_, checkerboardCols_);
    }
    bool exitStat = poseImage.poseEstimationImage(argv[1]);
    std::cout << "Process ended with status: " << exitStat << std::endl;
  }
  if (argc == 2) {
    CameraPose poseCamera(atof(argv[1]));
    std::cout << "Starting pose estimation for camera: " << std::endl;
    char defaultConfig;
    std::cout << "Do you want to use default chessboard inner rows(=9) and "
                 "inner columns(=6) configuration? Press y(es)/n(o)"
              << std::endl;
    std::cin >> defaultConfig;
    if (defaultConfig == 'n') {
      int checkerboardRows_, checkerboardCols_;
      std::cout << "Enter number of chessboard inner rows: ";
      std::cin >> checkerboardRows_;
      std::cout << std::endl;
      std::cout << "Enter number of chessboard inner columns: ";
      std::cin >> checkerboardCols_;
      std::cout << std::endl;
      poseCamera.setPatternSize(checkerboardRows_, checkerboardCols_);
    }
    bool exitStat = poseCamera.poseEstimationCamera();
    std::cout << "Process ended with status: " << exitStat << std::endl;
  }
  return 0;
}