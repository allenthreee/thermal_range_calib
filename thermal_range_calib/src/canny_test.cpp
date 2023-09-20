#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

// Detect edge by canny, and filter by edge length

int main(){
  cv::Mat src_img;
  src_img = cv::imread("/home/allen/calib_ws/data_demo/0thermal.png");
  cv::Mat canny_result = cv::Mat::zeros(src_img.size().height, src_img.size().width, CV_8UC1);
  int canny_threshold = 5;
  cv::Canny(src_img, canny_result, canny_threshold, canny_threshold * 3, 3,
            true);
  // ywy debug
  cv::Mat canny_img = cv::Mat::zeros(cv::Size(1280,720), CV_64FC1);
  cv::resize(canny_result, canny_img, canny_img.size(), 0, 0, cv::INTER_LINEAR);
  cv::imshow("canny test", canny_img);
  cv::waitKey(10000);
}
