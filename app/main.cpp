#include <iostream>
#include "CameraPose.hpp"

int main(int argc, char *argv[]) {
  CameraPose pose;

  if (argc != 2)
    std::cout << "ERROR: [Format expected: ./app/pose-app "
                 "../frames/frame_<number>.png]"
              << std::endl;
  std::cout << "Starting pose estimation: " << argv[1] << std::endl;
  pose.poseEstimation(argv[1]);
  std::cout << "Process ended: " << std::endl;
  return 0;
}