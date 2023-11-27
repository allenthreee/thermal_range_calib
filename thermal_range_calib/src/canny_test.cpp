#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "include/thermal_canny.hpp"



int main(){
  ThermalCanny my_thermal_canny("/home/allen/ooad_calib_ws/data_demo/ign_apartment.jpg");
  cv::Mat src_img;
  src_img = cv::imread("/home/allen/ooad_calib_ws/data_demo/ign_apartment.jpg");
  cv::Mat canny_result = cv::Mat::zeros(src_img.size().height, src_img.size().width, CV_8UC1);
  int canny_threshold = 50;
  std::cout << "before cv::Canny()" << std::endl;
  cv::Canny(src_img, canny_result, canny_threshold, canny_threshold * 3, 3,
            true);
  // ywy debug
  cv::Mat canny_img = cv::Mat::zeros(cv::Size(1280,720), CV_64FC1);
  cv::resize(canny_result, canny_img, canny_img.size(), 0, 0, cv::INTER_LINEAR);
  cv::imshow("canny test", canny_img);

  int old_ddepth = CV_16S;
  int k_size = 1+ 2*(2%5);
  std::cout << "k_size" << k_size << std::endl;
  int kernel_size = 5;
  int scale = 5;
  int delta = 0;
    // Reduce noise by blurring with a Gaussian filter ( kernel size = 3 )
  cv::GaussianBlur(src_img, src_img, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT );
  cv::cvtColor(src_img, src_img, cv::COLOR_BGR2GRAY);
  cv::Mat laplacian_edge = cv::Mat::zeros(src_img.size().height, src_img.size().width, CV_8UC1);
  cv::Laplacian(src_img, laplacian_edge, old_ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT );
  // converting back to CV_8U
  cv::Mat abs_laplacian_edge;
  cv::convertScaleAbs(laplacian_edge, abs_laplacian_edge);
  cv::Mat laplacian_img = cv::Mat::zeros(cv::Size(1280,720), CV_64FC1);
  cv::resize(abs_laplacian_edge, laplacian_img, canny_img.size(), 0, 0, cv::INTER_LINEAR);
  imshow("laplacian", laplacian_img);

cv::waitKey(100000);

}
