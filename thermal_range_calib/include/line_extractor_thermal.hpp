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


void ywy_canny_debug(cv::Mat &canny_result, int canny_threshold){
  // ywy debug
  std::cout << "ywy test canny_threshold" << canny_threshold << std::endl;
  cv::Mat canny_img = cv::Mat::zeros(cv::Size(1280,720), CV_64FC1);
  cv::resize(canny_result, canny_img, canny_img.size(), 0, 0, cv::INTER_LINEAR);
  cv::imshow("canny test", canny_img);
}

void fillEdgeImg(std::vector<std::vector<cv::Point>> contours, 
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
        // ywy comment, y=row, x=col, in matrix and image, xy order different
        edge_img.at<uchar>(-p.y, p.x) = 255;
      }
    }
  }
}

void fillEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud, cv::Mat edge_img){
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

// Detect edge by canny, and filter by edge length
pcl::PointCloud<pcl::PointXYZ>::Ptr thermalEdgeDetector(
    const int &canny_threshold, const int &edge_threshold,
    const cv::Mat &src_img) {

  std::cout << "YWY thermalEdgeDetector YWY" << std::endl;
  int gaussian_size = 5;
  // cv::GaussianBlur(src_img, src_img, cv::Size(gaussian_size, gaussian_size), 0,
  //                  0);
  int width = src_img.cols;
  int height = src_img.rows;
  cv::Mat edge_img;
  cv::Mat canny_result = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Canny(src_img, canny_result, canny_threshold, canny_threshold * 3, 3,
            true);
  // ywy_canny_debug(canny_result, canny_threshold);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

  edge_img = cv::Mat::zeros(height, width, CV_8UC1);
  fillEdgeImg(contours, edge_threshold, edge_img);      
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud =
  pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  fillEdgeCloud(edge_cloud, edge_img);

  edge_cloud->width = edge_cloud->points.size();
  edge_cloud->height = 1;
  cv::imshow("canny result", canny_result);
  cv::imshow("edge result", edge_img);
  cv::waitKey();

  
  return edge_cloud;
  // for (int x = 0; x < edge_img.cols; x++) {
  //   for (int y = 0; y < edge_img.rows; y++) {
  //     if (edge_img.at<uchar>(y, x) == 255) {
  //       pcl::PointXYZ p;
  //       p.x = x;
  //       p.y = -y;
  //       p.z = 0;
  //       edge_cloud->points.push_back(p);
  //     }
  //   }
  // }
}







  