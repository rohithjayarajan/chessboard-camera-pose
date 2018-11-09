/**
 *  Copyright 2018 rohithjayarajan
 *  @file    CameraPose.cpp
 *  @author  rohithjayarajan
 *  @date 11/1/2018
 *  @version 1.0
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