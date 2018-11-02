add_executable(calibration-app main.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

TARGET_LINK_LIBRARIES(calibration-app ${OpenCV_LIBS} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})