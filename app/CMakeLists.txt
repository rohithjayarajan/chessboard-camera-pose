add_executable(pose-app main.cpp CameraPose.cpp)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(pose-app ${OpenCV_LIBS} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})