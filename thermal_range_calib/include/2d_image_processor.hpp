#include "CustomMsg.h"
#include "common.h"
#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>

#include <pcl/common/common.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sstream>
#include <std_msgs/Header.h>
#include <stdio.h>
#include <string>
#include <chrono>
#include <thread>
#include <time.h>
#include <unordered_map>

#include "thermal_canny.hpp"

struct CameraIntrinsic
{
  /* data */
  float fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_, s_;
};


class ImageProcessor
{
private:
  /* data */
  // camera intrinsics
  // CameraIntrinsic camera_intrinsics_;  //image edge detection 这个类不需要相机内参
  // int image_width_, image_height_;
  // edge config
  int canny_gray_threshold_, canny_len_threshold_;
  int gaussian_size_;
  cv::Mat image_, gray_image_;
  // 存储2DImage边缘点的2D点云

public:
  int image_width_, image_height_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr image_edge_cloud_;

  ImageProcessor(const std::string &image_file, const std::string &calib_config_file);

  void LoadImage(const std::string &image_file);

  // void LoadCameraIntrinsic(const vector<double> cam_matrix, 
  //                          const vector<double> dist_coeffs);

  void LoadImageEdgeConfig(const string &calib_config_file);

  void DetectImageEdge();
  void FillEdgeImage(std::vector<std::vector<cv::Point>> contours, 
                 const int &edge_threshold,cv::Mat edge_img);
  void GenImageEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud, 
                         cv::Mat edge_img);

  ~ImageProcessor();
};

ImageProcessor::ImageProcessor(const std::string &image_file, const std::string &calib_config_file)
{
  LoadImage(image_file);
  LoadImageEdgeConfig(calib_config_file);
  DetectImageEdge();
  std::cout << "after DetectImageEdge() OOAD" << std::endl; 
}

void ImageProcessor::LoadImage(const std::string &image_file){
  image_ = cv::imread(image_file, cv::IMREAD_UNCHANGED);
  if (!image_.data) {
    std::string msg = "Can not load image from " + image_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  } else {
    std::string msg = "Sucessfully load image!";
    ROS_INFO_STREAM(msg.c_str());
  }
  image_width_ = image_.cols;
  image_height_ = image_.rows;
  // check rgb or gray
  if (image_.type() == CV_8UC1) {
    gray_image_ = image_;
  } else if (image_.type() == CV_8UC3) {
    cv::cvtColor(image_, gray_image_, cv::COLOR_BGR2GRAY);
  } else {
    std::string msg = "Unsupported image type, please use CV_8UC3 or CV_8UC1";
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
}

// void ImageProcessor::LoadCameraIntrinsic(const vector<double> camera_matrix, 
//                                          const vector<double> dist_coeffs){
//   camera_intrinsics_.fx_ = camera_matrix[0];
//   camera_intrinsics_.cx_ = camera_matrix[2];
//   camera_intrinsics_.fy_ = camera_matrix[4];
//   camera_intrinsics_.cy_ = camera_matrix[5];
//   camera_intrinsics_.k1_ = dist_coeffs[0];
//   camera_intrinsics_.k2_ = dist_coeffs[1];
//   camera_intrinsics_.p1_ = dist_coeffs[2];
//   camera_intrinsics_.p2_ = dist_coeffs[3];
//   camera_intrinsics_.k3_ = dist_coeffs[4];
// }

void ImageProcessor::LoadImageEdgeConfig(const std::string &calib_config_file){

  cv::FileStorage fSettings(calib_config_file, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    std::cerr << "Failed to open settings file at: " << calib_config_file
              << std::endl;
    exit(-1);
  } else {
    ROS_INFO("Sucessfully load calib config file");
  }
  canny_gray_threshold_ = fSettings["Canny.gray_threshold"];
  canny_len_threshold_ = fSettings["Canny.gray_threshold"];
  gaussian_size_ = fSettings["Canny.gaussian_size"];
}

void ImageProcessor::DetectImageEdge(){
  std::cout << "YWY ImageProcessor::DetectImageEdge() YWY" << std::endl;
  // int gaussian_size = 5;
  // cv::GaussianBlur(src_img, src_img, cv::Size(gaussian_size, gaussian_size), 0,
  //                  0);
  int width = gray_image_.cols;
  int height = gray_image_.rows;
  cv::Mat edge_img;
  cv::Mat canny_result = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Canny(gray_image_, canny_result, canny_gray_threshold_, canny_gray_threshold_ * 3, 3,
            true);
  // ywy_canny_debug(canny_result, canny_threshold);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
  std::cout << "we have:" << contours.size() << "contours" << std::endl;
  edge_img = cv::Mat::zeros(height, width, CV_8UC1);
  FillEdgeImage(contours, canny_len_threshold_, edge_img);      
  // for (size_t i = 0; i < contours.size(); i++) {
  //   if (contours[i].size() > edge_threshold) {
  //     cv::Mat debug_img = cv::Mat::zeros(height, width, CV_8UC1);
  //     for (size_t j = 0; j < contours[i].size(); j++) {
  //       pcl::PointXYZ p;
  //       p.x = contours[i][j].x;
  //       p.y = -contours[i][j].y;
  //       p.z = 0;
  //       edge_img.at<uchar>(-p.y, p.x) = 255;
  //     }
  //   }
  // }

  image_edge_cloud_ =
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  GenImageEdgeCloud(image_edge_cloud_, edge_img);
  // return edge_cloud;

}

void ImageProcessor::FillEdgeImage(std::vector<std::vector<cv::Point>> contours, 
                 const int &edge_threshold,cv::Mat edge_img){
  int width = edge_img.cols;
  int height = edge_img.rows;
  for (size_t i = 0; i < contours.size(); i++) {
    if (contours[i].size() > edge_threshold) {
      cv::Mat debug_img = cv::Mat::zeros(height, width, CV_8UC1);
      for (size_t j = 0; j < contours[i].size(); j++) {
        pcl::PointXYZ p;
        p.x = contours[i][j].x;
        p.y = -contours[i][j].y;
        p.z = 0;
        edge_img.at<uchar>(-p.y, p.x) = 255;
      }
    }
  }
}

void ImageProcessor::GenImageEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud, 
                                       cv::Mat edge_img){
  for (int x = 0; x < edge_img.cols; x++) {
    for (int y = 0; y < edge_img.rows; y++) {
      if (edge_img.at<uchar>(y, x) == 255) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = -y;
        p.z = 0;
        edge_cloud->points.push_back(p);
      }
    }
  }
  edge_cloud->width = edge_cloud->points.size();
  edge_cloud->height = 1;
}



ImageProcessor::~ImageProcessor()
{
}


