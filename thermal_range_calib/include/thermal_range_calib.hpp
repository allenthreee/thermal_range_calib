#ifndef THERMAL_RANGE_CALIB_HPP
#define THERMAL_RANGE_CALIB_HPP
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

#include "line_extractor_thermal.hpp"
#include "line_extractor_range.hpp"
#include "misc.hpp"


#define calib
#define online
class Calibration {
public:
  ros::NodeHandle nh_;
  ros::Publisher rgb_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1);
  ros::Publisher ywy_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("ywy_raw_cloud", 1);
  ros::Publisher init_rgb_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("init_rgb_cloud", 1);
  ros::Publisher planner_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("planner_cloud", 1);

  ros::Publisher ywy_voxel_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("ywy_voxel_cloud", 1);

  ros::Publisher line_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("line_cloud", 1);
  ros::Publisher image_pub_ =
      nh_.advertise<sensor_msgs::Image>("camera_image", 1);
  enum ProjectionType { DEPTH, INTENSITY, BOTH };
  enum Direction { UP, DOWN, LEFT, RIGHT };
  std::string lidar_topic_name_ = "";
  std::string image_topic_name_ = "";

  int rgb_edge_minLen_ = 200;
  int rgb_canny_threshold_ = 20;
  int min_depth_ = 2.5;
  int max_depth_ = 50;
  // int min_depth_ = 0.5;
  // int max_depth_ = 10;
  int plane_max_size_ = 5;
  float detect_line_threshold_ = 0.02;
  int line_number_ = 0;
  int color_intensity_threshold_ = 5;
  Eigen::Vector3d adjust_euler_angle_;
  Calibration(const std::string &image_file, const std::string &pcd_file,
              const std::string &calib_config_file);
  void loadImgAndPointcloud(const std::string bag_path,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &origin_cloud,
                            cv::Mat &rgb_img);
  bool loadCameraConfig(const std::string &camera_file);
  bool loadCalibConfig(const std::string &config_file);
  bool loadConfig(const std::string &configFile);
  bool checkFov(const cv::Point2d &p);
  void colorCloud(const Vector6d &extrinsic_params, const int density,
                  const cv::Mat &rgb_img,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_cloud);

  cv::Mat fillImg(const cv::Mat &input_img, const Direction first_direct,
                  const Direction second_direct);

  void buildVPnp(const Vector6d &extrinsic_params, const int dis_threshold,
            const bool show_residual,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cam_edge_cloud_2d,
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d,
            std::vector<pnl_data> &pnp_list);

  cv::Mat getConnectImg(const int dis_threshold,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &rgb_edge_cloud,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &depth_edge_cloud);
  cv::Mat getProjectionImg(const Vector6d &extrinsic_params);
  void initVoxel(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                 const float voxel_size,
                 std::unordered_map<VOXEL_LOC, Voxel *> &voxel_map);
  void LiDAREdgeExtraction(
      const std::unordered_map<VOXEL_LOC, Voxel *> &voxel_map,
      const float ransac_dis_thre, const int plane_size_threshold,
      pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d);
  void calcDirection(const std::vector<Eigen::Vector2d> &points,
                     Eigen::Vector2d &direction);
  void calcResidual(const Vector6d &extrinsic_params,
                    const std::vector<pnl_data> vpnp_list,
                    std::vector<float> &residual_list);
  void calcCovarance(const Vector6d &extrinsic_params,
                     const pnl_data &vpnp_point, const float pixel_inc,
                     const float range_inc, const float degree_inc,
                     Eigen::Matrix2f &covarance);
  // 相机内参
  float fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_, s_;
  int width_, height_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat init_extrinsic_;

  int is_use_custom_msg_;
  float voxel_size_ = 1.0;
  float down_sample_size_ = 0.02;
  float ransac_dis_threshold_ = 0.02;
  float plane_size_threshold_ = 60;
  float theta_min_;
  float theta_max_;
  float direction_theta_min_;
  float direction_theta_max_;
  float min_line_dis_threshold_ = 0.03;
  float max_line_dis_threshold_ = 0.06;

  cv::Mat rgb_image_;
  cv::Mat image_;
  cv::Mat grey_image_;
  // 裁剪后的灰度图像
  cv::Mat cut_grey_image_;

  // 初始旋转矩阵
  Eigen::Matrix3d init_rotation_matrix_;
  // 初始平移向量
  Eigen::Vector3d init_translation_vector_;

  // 存储从pcd/bag处获取的原始点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_lidar_cloud_;

  // 存储平面交接点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_line_cloud_;
  std::vector<int> plane_line_number_;
  // 存储RGB图像边缘点的2D点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr rgb_edge_cloud_;
  // 存储LiDAR Depth/Intensity图像边缘点的2D点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_edge_cloud_;
};

Calibration::Calibration(const std::string &image_file,
                         const std::string &pcd_file,
                         const std::string &calib_config_file) {

  loadCalibConfig(calib_config_file);

  image_ = cv::imread(image_file, cv::IMREAD_UNCHANGED);
  if (!image_.data) {
    std::string msg = "Can not load image from " + image_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  } else {
    std::string msg = "Sucessfully load image!";
    ROS_INFO_STREAM(msg.c_str());
  }
  width_ = image_.cols;
  height_ = image_.rows;
  // check rgb or gray
  if (image_.type() == CV_8UC1) {
    grey_image_ = image_;
  } else if (image_.type() == CV_8UC3) {
    cv::cvtColor(image_, grey_image_, cv::COLOR_BGR2GRAY);
  } else {
    std::string msg = "Unsupported image type, please use CV_8UC3 or CV_8UC1";
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  cv::Mat edge_image;

  // edgeDetector(rgb_canny_threshold_, rgb_edge_minLen_, grey_image_, edge_image, rgb_edge_cloud_);
  rgb_edge_cloud_ = thermalEdgeDetector(rgb_canny_threshold_, rgb_edge_minLen_, grey_image_);


  std::string msg = "Sucessfully extract edge from image, edge size:" +
                    std::to_string(rgb_edge_cloud_->size());
  ROS_INFO_STREAM(msg.c_str());

  raw_lidar_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  ROS_INFO_STREAM("Loading point cloud from pcd file.");
  if (!pcl::io::loadPCDFile(pcd_file, *raw_lidar_cloud_)) {
    // down_sampling_voxel(*raw_lidar_cloud_, 0.02);
    std::string msg = "Sucessfully load pcd, pointcloud size: " +
                      std::to_string(raw_lidar_cloud_->size());
    // ywy
    ROS_INFO_STREAM(msg.c_str());
    // YWY cancel comment =====================
    
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*raw_lidar_cloud_, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;
    // ywy
  } else {
    std::string msg = "Unable to load " + pcd_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }

  Eigen::Vector3d lwh(50, 50, 30);
  Eigen::Vector3d origin(0, -25, -10);
  std::vector<VoxelGrid> voxel_list;
  std::unordered_map<VOXEL_LOC, Voxel *> voxel_map;
  // initVoxel(raw_lidar_cloud_, voxel_size_, voxel_map);
  ywy_initVoxel(raw_lidar_cloud_, voxel_size_, voxel_map);
  LiDAREdgeExtraction(voxel_map, ransac_dis_threshold_, plane_size_threshold_,
                      plane_line_cloud_);
};

bool Calibration::loadCameraConfig(const std::string &camera_file) {
  cv::FileStorage cameraSettings(camera_file, cv::FileStorage::READ);
  if (!cameraSettings.isOpened()) {
    std::cerr << "Failed to open camera settings file at " << camera_file
              << std::endl;
    exit(-1);
  }
  width_ = cameraSettings["Camera.width"];
  height_ = cameraSettings["Camera.height"];
  cameraSettings["CameraMat"] >> camera_matrix_;
  cameraSettings["DistCoeffs"] >> dist_coeffs_;
  fx_ = camera_matrix_.at<double>(0, 0);
  cx_ = camera_matrix_.at<double>(0, 2);
  fy_ = camera_matrix_.at<double>(1, 1);
  cy_ = camera_matrix_.at<double>(1, 2);
  k1_ = dist_coeffs_.at<double>(0, 0);
  k2_ = dist_coeffs_.at<double>(0, 1);
  p1_ = dist_coeffs_.at<double>(0, 2);
  p2_ = dist_coeffs_.at<double>(0, 3);
  k3_ = dist_coeffs_.at<double>(0, 4);
  std::cout << "Camera Matrix: " << std::endl << camera_matrix_ << std::endl;
  std::cout << "Distortion Coeffs: " << std::endl << dist_coeffs_ << std::endl;
  return true;
};

// 可以把calibConfig写成一个结构体，看看哪些是必要的，哪些是重复的
bool Calibration::loadCalibConfig(const std::string &config_file) {
  cv::FileStorage fSettings(config_file, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    std::cerr << "Failed to open settings file at: " << config_file
              << std::endl;
    exit(-1);
  } else {
    ROS_INFO("Sucessfully load calib config file");
  }
  fSettings["ExtrinsicMat"] >> init_extrinsic_;
  init_rotation_matrix_ << init_extrinsic_.at<double>(0, 0),
      init_extrinsic_.at<double>(0, 1), init_extrinsic_.at<double>(0, 2),
      init_extrinsic_.at<double>(1, 0), init_extrinsic_.at<double>(1, 1),
      init_extrinsic_.at<double>(1, 2), init_extrinsic_.at<double>(2, 0),
      init_extrinsic_.at<double>(2, 1), init_extrinsic_.at<double>(2, 2);
  init_translation_vector_ << init_extrinsic_.at<double>(0, 3),
      init_extrinsic_.at<double>(1, 3), init_extrinsic_.at<double>(2, 3);
  rgb_canny_threshold_ = fSettings["Canny.gray_threshold"];
  rgb_edge_minLen_ = fSettings["Canny.len_threshold"];
  voxel_size_ = fSettings["Voxel.size"];
  down_sample_size_ = fSettings["Voxel.down_sample_size"];
  plane_size_threshold_ = fSettings["Plane.min_points_size"];
  plane_max_size_ = fSettings["Plane.max_size"];
  ransac_dis_threshold_ = fSettings["Ransac.dis_threshold"];
  min_line_dis_threshold_ = fSettings["Edge.min_dis_threshold"];
  max_line_dis_threshold_ = fSettings["Edge.max_dis_threshold"];
  theta_min_ = fSettings["Plane.normal_theta_min"];
  theta_max_ = fSettings["Plane.normal_theta_max"];
  theta_min_ = cos(DEG2RAD(theta_min_));
  theta_max_ = cos(DEG2RAD(theta_max_));
  direction_theta_min_ = cos(DEG2RAD(30.0));
  direction_theta_max_ = cos(DEG2RAD(150.0));
  color_intensity_threshold_ = fSettings["Color.intensity_threshold"];
  return true;
};

// Color the point cloud by rgb image using given extrinsic
void Calibration::colorCloud(
    const Vector6d &extrinsic_params, const int density,
    const cv::Mat &input_image,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_cloud) {
  cv::Mat rgb_img;
  if (input_image.type() == CV_8UC3) {
    rgb_img = input_image;
  } else if (input_image.type() == CV_8UC1) {
    cv::cvtColor(input_image, rgb_img, cv::COLOR_GRAY2BGR);
  }
  std::vector<cv::Point3f> pts_3d;
  for (size_t i = 0; i < lidar_cloud->size(); i += density) {
    pcl::PointXYZI point = lidar_cloud->points[i];
    float depth = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    if (depth > 2 && depth < 50 &&
        point.intensity >= color_intensity_threshold_) {
      pts_3d.emplace_back(cv::Point3f(point.x, point.y, point.z));
    }
  }
  Eigen::AngleAxisd rotation_vector3;
  rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cv::Mat distortion_coeff =
      (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
  cv::Mat r_vec =
      (cv::Mat_<double>(3, 1)
           << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
  cv::Mat t_vec = (cv::Mat_<double>(3, 1) << extrinsic_params[3],
                   extrinsic_params[4], extrinsic_params[5]);
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                    pts_2d);
  int image_rows = rgb_img.rows;
  int image_cols = rgb_img.cols;
  color_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (size_t i = 0; i < pts_2d.size(); i++) {
    if (pts_2d[i].x >= 0 && pts_2d[i].x < image_cols && pts_2d[i].y >= 0 &&
        pts_2d[i].y < image_rows) {
      cv::Scalar color =
          rgb_img.at<cv::Vec3b>((int)pts_2d[i].y, (int)pts_2d[i].x);
      if (color[0] == 0 && color[1] == 0 && color[2] == 0) {
        continue;
      }
      if (pts_3d[i].x > 100) {
        continue;
      }
      pcl::PointXYZRGB p;
      p.x = pts_3d[i].x;
      p.y = pts_3d[i].y;
      p.z = pts_3d[i].z;
      // p.a = 255;
      p.b = color[0];
      p.g = color[1];
      p.r = color[2];
      color_cloud->points.push_back(p);
    }
  }
  color_cloud->width = color_cloud->points.size();
  color_cloud->height = 1;
}


// 填补雷达深度图像
cv::Mat Calibration::fillImg(const cv::Mat &input_img,
                             const Direction first_direct,
                             const Direction second_direct) {
  cv::Mat fill_img = input_img.clone();
  for (int y = 2; y < input_img.rows - 2; y++) {
    for (int x = 2; x < input_img.cols - 2; x++) {
      if (input_img.at<uchar>(y, x) == 0) {
        if (input_img.at<uchar>(y - 1, x) != 0) {
          fill_img.at<uchar>(y, x) = input_img.at<uchar>(y - 1, x);
        } else {
          if ((input_img.at<uchar>(y, x - 1)) != 0) {
            fill_img.at<uchar>(y, x) = input_img.at<uchar>(y, x - 1);
          }
        }
      } else {
        int left_depth = input_img.at<uchar>(y, x - 1);
        int right_depth = input_img.at<uchar>(y, x + 1);
        int up_depth = input_img.at<uchar>(y + 1, x);
        int down_depth = input_img.at<uchar>(y - 1, x);
        int current_depth = input_img.at<uchar>(y, x);
        if ((current_depth - left_depth) > 5 &&
            (current_depth - right_depth) > 5 && left_depth != 0 &&
            right_depth != 0) {
          fill_img.at<uchar>(y, x) = (left_depth + right_depth) / 2;
        } else if ((current_depth - up_depth) > 5 &&
                   (current_depth - down_depth) > 5 && up_depth != 0 &&
                   down_depth != 0) {
          fill_img.at<uchar>(y, x) = (up_depth + down_depth) / 2;
        }
      }
    }
  }
  return fill_img;
}

cv::Mat Calibration::getConnectImg(
    const int dis_threshold,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &rgb_edge_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &depth_edge_cloud) {
  cv::Mat connect_img = cv::Mat::zeros(height_, width_, CV_8UC3);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree->setInputCloud(rgb_edge_cloud);
  tree_cloud = rgb_edge_cloud;
  for (size_t i = 0; i < depth_edge_cloud->points.size(); i++) {
    cv::Point2d p2(depth_edge_cloud->points[i].x,
                   -depth_edge_cloud->points[i].y);
    if (checkFov(p2)) {
      pcl::PointXYZ p = depth_edge_cloud->points[i];
      search_cloud->points.push_back(p);
    }
  }

  int line_count = 0;
  // 指定近邻个数
  int K = 1;
  // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  for (size_t i = 0; i < search_cloud->points.size(); i++) {
    pcl::PointXYZ searchPoint = search_cloud->points[i];
    if (kdtree->nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                               pointNKNSquaredDistance) > 0) {
      for (int j = 0; j < K; j++) {
        float distance = sqrt(
            pow(searchPoint.x - tree_cloud->points[pointIdxNKNSearch[j]].x, 2) +
            pow(searchPoint.y - tree_cloud->points[pointIdxNKNSearch[j]].y, 2));
        if (distance < dis_threshold) {
          cv::Scalar color = cv::Scalar(0, 255, 0);
          line_count++;
          if ((line_count % 3) == 0) {
            cv::line(connect_img,
                     cv::Point(search_cloud->points[i].x,
                               -search_cloud->points[i].y),
                     cv::Point(tree_cloud->points[pointIdxNKNSearch[j]].x,
                               -tree_cloud->points[pointIdxNKNSearch[j]].y),
                     color, 1);
          }
        }
      }
    }
  }
  for (size_t i = 0; i < rgb_edge_cloud->size(); i++) {
    connect_img.at<cv::Vec3b>(-rgb_edge_cloud->points[i].y,
                              rgb_edge_cloud->points[i].x)[0] = 255;
    connect_img.at<cv::Vec3b>(-rgb_edge_cloud->points[i].y,
                              rgb_edge_cloud->points[i].x)[1] = 0;
    connect_img.at<cv::Vec3b>(-rgb_edge_cloud->points[i].y,
                              rgb_edge_cloud->points[i].x)[2] = 0;
  }
  for (size_t i = 0; i < search_cloud->size(); i++) {
    connect_img.at<cv::Vec3b>(-search_cloud->points[i].y,
                              search_cloud->points[i].x)[0] = 0;
    connect_img.at<cv::Vec3b>(-search_cloud->points[i].y,
                              search_cloud->points[i].x)[1] = 0;
    connect_img.at<cv::Vec3b>(-search_cloud->points[i].y,
                              search_cloud->points[i].x)[2] = 255;
  }
  int expand_size = 2;
  cv::Mat expand_edge_img;
  expand_edge_img = connect_img.clone();
  for (int x = expand_size; x < connect_img.cols - expand_size; x++) {
    for (int y = expand_size; y < connect_img.rows - expand_size; y++) {
      if (connect_img.at<cv::Vec3b>(y, x)[0] == 255) {
        for (int xx = x - expand_size; xx <= x + expand_size; xx++) {
          for (int yy = y - expand_size; yy <= y + expand_size; yy++) {
            expand_edge_img.at<cv::Vec3b>(yy, xx)[0] = 255;
            expand_edge_img.at<cv::Vec3b>(yy, xx)[1] = 0;
            expand_edge_img.at<cv::Vec3b>(yy, xx)[2] = 0;
          }
        }
      } else if (connect_img.at<cv::Vec3b>(y, x)[2] == 255) {
        for (int xx = x - expand_size; xx <= x + expand_size; xx++) {
          for (int yy = y - expand_size; yy <= y + expand_size; yy++) {
            expand_edge_img.at<cv::Vec3b>(yy, xx)[0] = 0;
            expand_edge_img.at<cv::Vec3b>(yy, xx)[1] = 0;
            expand_edge_img.at<cv::Vec3b>(yy, xx)[2] = 255;
          }
        }
      }
    }
  }
  return connect_img;
}

bool Calibration::checkFov(const cv::Point2d &p) {
  if (p.x > 0 && p.x < width_ && p.y > 0 && p.y < height_) {
    return true;
  } else {
    return false;
  }
}

void Calibration::LiDAREdgeExtraction(
    const std::unordered_map<VOXEL_LOC, Voxel *> &voxel_map,
    const float ransac_dis_thre, const int plane_size_threshold,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d) {
  ROS_INFO_STREAM("Extracting Lidar Edge");
  ros::Rate loop(5000);
  lidar_line_cloud_3d =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {

    // if (iter->second->cloud->size() > 50) {
    // ywy change this cloud size to 10, origin size > 50
    if (iter->second->cloud->size() > 10) {
      std::vector<Plane> plane_list = planeFittingInVoxel(ransac_dis_thre, plane_size_threshold, iter->second->cloud, planner_cloud_pub_);
      
      std::cout << "YWY Plane number: " << plane_list.size() << std::endl;

      std::vector<pcl::PointCloud<pcl::PointXYZI>> line_cloud_list;
      calcLineFromVoxel(plane_list, voxel_size_, iter->second->voxel_origin,
                        min_line_dis_threshold_, max_line_dis_threshold_, 
                        theta_min_, theta_max_, plane_max_size_,
                        line_cloud_list);
      // ouster 5,normal 3
      if (line_cloud_list.size() > 0 && line_cloud_list.size() <= 8) {

        for (size_t cloud_index = 0; cloud_index < line_cloud_list.size();
             cloud_index++) {
          for (size_t i = 0; i < line_cloud_list[cloud_index].size(); i++) {
            pcl::PointXYZI p = line_cloud_list[cloud_index].points[i];
            plane_line_cloud_->points.push_back(p);
            sensor_msgs::PointCloud2 pub_cloud;
            pcl::toROSMsg(line_cloud_list[cloud_index], pub_cloud);
            pub_cloud.header.frame_id = "world";
            line_cloud_pub_.publish(pub_cloud);
            loop.sleep();
            plane_line_number_.push_back(line_number_);
          }
          line_number_++;
        }
      }
    }
  }
  std::cout << "we got " << line_number_ << " LiDAR edges" << std::endl;
}

void Calibration::buildVPnp(
    const Vector6d &extrinsic_params, const int dis_threshold,
    const bool show_residual,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cam_edge_cloud_2d,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d,
    std::vector<pnl_data> &pnp_list) {
  pnp_list.clear();
  std::vector<std::vector<std::vector<pcl::PointXYZI>>> img_pts_container;
  for (int y = 0; y < height_; y++) {
    std::vector<std::vector<pcl::PointXYZI>> row_pts_container;
    for (int x = 0; x < width_; x++) {
      std::vector<pcl::PointXYZI> col_pts_container;
      row_pts_container.push_back(col_pts_container);
    }
    img_pts_container.push_back(row_pts_container);
  }
  std::vector<cv::Point3d> pts_3d;
  Eigen::AngleAxisd rotation_vector3;
  rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());

  for (size_t i = 0; i < lidar_line_cloud_3d->size(); i++) {
    pcl::PointXYZI point_3d = lidar_line_cloud_3d->points[i];
    pts_3d.emplace_back(cv::Point3d(point_3d.x, point_3d.y, point_3d.z));
  }
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cv::Mat distortion_coeff =
      (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
  cv::Mat r_vec =
      (cv::Mat_<double>(3, 1)
           << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
  cv::Mat t_vec = (cv::Mat_<double>(3, 1) << extrinsic_params[3],
                   extrinsic_params[4], extrinsic_params[5]);
  // project 3d-points into image view
  std::vector<cv::Point2d> pts_2d;
  // debug
  // std::cout << "camera_matrix:" << camera_matrix << std::endl;
  // std::cout << "distortion_coeff:" << distortion_coeff << std::endl;
  // std::cout << "r_vec:" << r_vec << std::endl;
  // std::cout << "t_vec:" << t_vec << std::endl;
  // std::cout << "pts 3d size:" << pts_3d.size() << std::endl;
  cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                    pts_2d);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_edge_cloud_2d(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> line_edge_cloud_2d_number;
  for (size_t i = 0; i < pts_2d.size(); i++) {
    pcl::PointXYZ p;
    p.x = pts_2d[i].x;
    p.y = -pts_2d[i].y;
    p.z = 0;
    pcl::PointXYZI pi_3d;
    pi_3d.x = pts_3d[i].x;
    pi_3d.y = pts_3d[i].y;
    pi_3d.z = pts_3d[i].z;
    pi_3d.intensity = 1;
    if (p.x > 0 && p.x < width_ && pts_2d[i].y > 0 && pts_2d[i].y < height_) {
      if (img_pts_container[pts_2d[i].y][pts_2d[i].x].size() == 0) {
        line_edge_cloud_2d->points.push_back(p);
        line_edge_cloud_2d_number.push_back(plane_line_number_[i]);
        img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
      } else {
        img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
      }
    }
  }
  if (show_residual) {
    cv::Mat residual_img =
        getConnectImg(dis_threshold, cam_edge_cloud_2d, line_edge_cloud_2d);
    cv::Mat small_residual_img = cv::Mat::zeros(cv::Size(1280,720), CV_64FC1);
    cv::resize(residual_img, small_residual_img, small_residual_img.size(), 0, 0, cv::INTER_LINEAR);
    // cv::imshow("residual", small_residual_img);
    cv::waitKey(100);
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_lidar(
      new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud_lidar =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree->setInputCloud(cam_edge_cloud_2d);
  kdtree_lidar->setInputCloud(line_edge_cloud_2d);
  tree_cloud = cam_edge_cloud_2d;
  tree_cloud_lidar = line_edge_cloud_2d;
  search_cloud = line_edge_cloud_2d;
  // 指定近邻个数
  int K = 5;
  // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  std::vector<int> pointIdxNKNSearchLidar(K);
  std::vector<float> pointNKNSquaredDistanceLidar(K);
  int match_count = 0;
  double mean_distance;
  int line_count = 0;
  std::vector<cv::Point2d> lidar_2d_list;
  std::vector<cv::Point2d> img_2d_list;
  std::vector<Eigen::Vector2d> camera_direction_list;
  std::vector<Eigen::Vector2d> lidar_direction_list;
  std::vector<int> lidar_2d_number;
  for (size_t i = 0; i < search_cloud->points.size(); i++) {
    pcl::PointXYZ searchPoint = search_cloud->points[i];
    if ((kdtree->nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) &&
        (kdtree_lidar->nearestKSearch(searchPoint, K, pointIdxNKNSearchLidar,
                                      pointNKNSquaredDistanceLidar) > 0)) {
      bool dis_check = true;
      for (int j = 0; j < K; j++) {
        float distance = sqrt(
            pow(searchPoint.x - tree_cloud->points[pointIdxNKNSearch[j]].x, 2) +
            pow(searchPoint.y - tree_cloud->points[pointIdxNKNSearch[j]].y, 2));
        if (distance > dis_threshold) {
          dis_check = false;
        }
      }
      if (dis_check) {
        cv::Point p_l_2d(search_cloud->points[i].x, -search_cloud->points[i].y);
        cv::Point p_c_2d(tree_cloud->points[pointIdxNKNSearch[0]].x,
                         -tree_cloud->points[pointIdxNKNSearch[0]].y);
        Eigen::Vector2d direction_cam(0, 0);
        std::vector<Eigen::Vector2d> points_cam;
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
          Eigen::Vector2d p(tree_cloud->points[pointIdxNKNSearch[i]].x,
                            -tree_cloud->points[pointIdxNKNSearch[i]].y);
          points_cam.push_back(p);
        }
        calcDirection(points_cam, direction_cam);
        Eigen::Vector2d direction_lidar(0, 0);
        std::vector<Eigen::Vector2d> points_lidar;
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
          Eigen::Vector2d p(
              tree_cloud_lidar->points[pointIdxNKNSearchLidar[i]].x,
              -tree_cloud_lidar->points[pointIdxNKNSearchLidar[i]].y);
          points_lidar.push_back(p);
        }
        calcDirection(points_lidar, direction_lidar);
        // direction.normalize();
        if (checkFov(p_l_2d)) {
          lidar_2d_list.push_back(p_l_2d);
          img_2d_list.push_back(p_c_2d);
          camera_direction_list.push_back(direction_cam);
          lidar_direction_list.push_back(direction_lidar);
          lidar_2d_number.push_back(line_edge_cloud_2d_number[i]);
        }
      }
    }
  }
  for (size_t i = 0; i < lidar_2d_list.size(); i++) {
    int y = lidar_2d_list[i].y;
    int x = lidar_2d_list[i].x;
    int pixel_points_size = img_pts_container[y][x].size();
    if (pixel_points_size > 0) {
      pnl_data pnp;
      pnp.x = 0;
      pnp.y = 0;
      pnp.z = 0;
      pnp.u = img_2d_list[i].x;
      pnp.v = img_2d_list[i].y;
      for (size_t j = 0; j < pixel_points_size; j++) {
        pnp.x += img_pts_container[y][x][j].x;
        pnp.y += img_pts_container[y][x][j].y;
        pnp.z += img_pts_container[y][x][j].z;
      }
      pnp.x = pnp.x / pixel_points_size;
      pnp.y = pnp.y / pixel_points_size;
      pnp.z = pnp.z / pixel_points_size;
      pnp.direction = camera_direction_list[i];
      pnp.direction_lidar = lidar_direction_list[i];
      pnp.number = lidar_2d_number[i];
      float theta = pnp.direction.dot(pnp.direction_lidar);
      if (theta > direction_theta_min_ || theta < direction_theta_max_) {
        pnp_list.push_back(pnp);
      }
    }
  }
}

void Calibration::loadImgAndPointcloud(
    const std::string path, pcl::PointCloud<pcl::PointXYZI>::Ptr &origin_cloud,
    cv::Mat &rgb_img) {
  origin_cloud =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  std::fstream file_;
  file_.open(path, ios::in);
  if (!file_) {
    cout << "File " << path << " does not exit" << endl;
    return;
  }
  ROS_INFO("Loading the rosbag %s", path.c_str());
  rosbag::Bag bag;
  try {
    bag.open(path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return;
  }

  std::vector<string> lidar_topic;
  lidar_topic.push_back(lidar_topic_name_);
  rosbag::View view(bag, rosbag::TopicQuery(lidar_topic));

  int cloudCount = 0;
  for (const rosbag::MessageInstance &m : view) {
    if (is_use_custom_msg_) {
      livox_ros_driver::CustomMsg livox_cloud_msg =
          *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type

      for (uint i = 0; i < livox_cloud_msg.point_num; ++i) {
        pcl::PointXYZI p;
        p.x = livox_cloud_msg.points[i].x;
        p.y = livox_cloud_msg.points[i].y;
        p.z = livox_cloud_msg.points[i].z;
        p.intensity = livox_cloud_msg.points[i].reflectivity;
        origin_cloud->points.push_back(p);
      }
    } else {
      sensor_msgs::PointCloud2 livox_cloud;
      livox_cloud =
          *(m.instantiate<sensor_msgs::PointCloud2>()); // message type
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(livox_cloud, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, cloud);
      for (uint i = 0; i < cloud.size(); ++i) {
        origin_cloud->points.push_back(cloud.points[i]);
      }
    }

    ++cloudCount;
    // maxinum msg num 1000
    // if (cloudCount > 1000) {
    //   break;
    // }
  }

  

  std::vector<string> img_topic;
  img_topic.push_back(image_topic_name_);
  rosbag::View img_view(bag, rosbag::TopicQuery(img_topic));
  int cnt = 0;
  for (const rosbag::MessageInstance &m : img_view) {
    cnt++;
    if (cnt == 1) {
      sensor_msgs::Image image;
      image = *(m.instantiate<sensor_msgs::Image>()); // message type
      cv_bridge::CvImagePtr img_ptr =
          cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      img_ptr->image.copyTo(rgb_img);
    }
  }
  ROS_INFO("Sucessfully load Point Cloud and Image");
}

void Calibration::calcDirection(const std::vector<Eigen::Vector2d> &points,
                                Eigen::Vector2d &direction) {
  Eigen::Vector2d mean_point(0, 0);
  for (size_t i = 0; i < points.size(); i++) {
    mean_point(0) += points[i](0);
    mean_point(1) += points[i](1);
  }
  mean_point(0) = mean_point(0) / points.size();
  mean_point(1) = mean_point(1) / points.size();
  Eigen::Matrix2d S;
  S << 0, 0, 0, 0;
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Matrix2d s =
        (points[i] - mean_point) * (points[i] - mean_point).transpose();
    S += s;
  }
  Eigen::EigenSolver<Eigen::Matrix<double, 2, 2>> es(S);
  Eigen::MatrixXcd evecs = es.eigenvectors();
  Eigen::MatrixXcd evals = es.eigenvalues();
  Eigen::MatrixXd evalsReal;
  evalsReal = evals.real();
  Eigen::MatrixXf::Index evalsMax;
  evalsReal.rowwise().sum().maxCoeff(&evalsMax); //得到最大特征值的位置
  direction << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax);
}

cv::Mat Calibration::getProjectionImg(const Vector6d &extrinsic_params) {
  cv::Mat depth_projection_img;
  projectPointCloud2Img(extrinsic_params, raw_lidar_cloud_, misc_DEPTH, false,
             depth_projection_img, fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_, width_, height_);
  cv::Mat small_depth_projection_img = cv::Mat::zeros(cv::Size(1280,720), CV_64FC1);
  cv::resize(depth_projection_img, small_depth_projection_img, small_depth_projection_img.size(), 0, 0, cv::INTER_LINEAR);
  // cv::imshow("depth_projection_img", small_depth_projection_img);
  // cv::imshow("depth_projection_img", depth_projection_img);
  std::cout << "ywy check img height_:" << height_ << std::endl;
  std::cout << "ywy check img width_:" << width_ << std::endl;
  cv::Mat map_img = cv::Mat::zeros(height_, width_, CV_8UC3);
  for (int x = 0; x < map_img.cols; x++) {
    for (int y = 0; y < map_img.rows; y++) {
      uint8_t r, g, b;
      float norm = depth_projection_img.at<uchar>(y, x) / 256.0;
      mapJet(norm, 0, 1, r, g, b);
      map_img.at<cv::Vec3b>(y, x)[0] = b;
      map_img.at<cv::Vec3b>(y, x)[1] = g;
      map_img.at<cv::Vec3b>(y, x)[2] = r;
    }
  }
  cv::Mat merge_img;
  if (image_.type() == CV_8UC3) {
    merge_img = 0.5 * map_img + 0.8 * image_;
  } else {
    cv::Mat src_rgb;
    cv::cvtColor(image_, src_rgb, cv::COLOR_GRAY2BGR);
    merge_img = 0.5 * map_img + 0.8 * src_rgb;
  }
  return merge_img;
}

void Calibration::calcResidual(const Vector6d &extrinsic_params,
                               const std::vector<pnl_data> vpnp_list,
                               std::vector<float> &residual_list) {
  residual_list.clear();
  Eigen::Vector3d euler_angle(extrinsic_params[0], extrinsic_params[1],
                              extrinsic_params[2]);
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
  Eigen::Vector3d transation(extrinsic_params[3], extrinsic_params[4],
                             extrinsic_params[5]);
  for (size_t i = 0; i < vpnp_list.size(); i++) {
    Eigen::Vector2d residual;
    Eigen::Matrix2f var;
    calcCovarance(extrinsic_params, vpnp_list[i], 1, 0.02, 0.05, var);
    pnl_data vpnp_point = vpnp_list[i];
    float fx = fx_;
    float cx = cx_;
    float fy = fy_;
    float cy = cy_;
    Eigen::Matrix3d inner;
    inner << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    Eigen::Vector4d distor;
    distor << k1_, k2_, p1_, p2_;
    Eigen::Vector3d p_l(vpnp_point.x, vpnp_point.y, vpnp_point.z);
    Eigen::Vector3d p_c = rotation_matrix * p_l + transation;
    Eigen::Vector3d p_2 = inner * p_c;
    float uo = p_2[0] / p_2[2];
    float vo = p_2[1] / p_2[2];
    float xo = (uo - cx) / fx;
    float yo = (vo - cy) / fy;
    float r2 = xo * xo + yo * yo;
    float r4 = r2 * r2;
    float distortion = 1.0 + distor[0] * r2 + distor[1] * r4;
    float xd = xo * distortion + (distor[2] * xo * yo + distor[2] * xo * yo) +
               distor[3] * (r2 + xo * xo + xo * xo);
    float yd = yo * distortion + distor[2] * xo * yo + distor[2] * xo * yo +
               distor[2] * (r2 + yo * yo + yo * yo);
    float ud = fx * xd + cx;
    float vd = fy * yd + cy;
    residual[0] = ud - vpnp_point.u;
    residual[1] = vd - vpnp_point.v;
    // if (vpnp_point.direction(0) == 0 && vpnp_point.direction(1) == 0) {
    //   residual[0] = ud - vpnp_point.u;
    //   residual[1] = vd - vpnp_point.v;
    // } else {
    //   residual[0] = ud - vpnp_point.u;
    //   residual[1] = vd - vpnp_point.v;
    //   Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    //   Eigen::Vector2d n = vpnp_point.direction;
    //   Eigen::Matrix2d V = n * n.transpose();
    //   Eigen::Matrix2d R;
    //   R << residual[0], 0, 0, residual[1];
    //   V = I - V;
    //   R = V * R * V.transpose();
    //   residual[0] = R(0, 0);
    //   residual[1] = R(1, 1);
    // }
    // Eigen::Vector2d v(-vpnp_point.direction(1), vpnp_point.direction(0));
    // if (v(0) < 0) {
    //   v = -v;
    // }
    float cost = sqrt(residual[0] * residual[0] + residual[1] * residual[1]);
    residual_list.push_back(cost);
  }
}

void Calibration::calcCovarance(const Vector6d &extrinsic_params,
                                const pnl_data &vpnp_point,
                                const float pixel_inc, const float range_inc,
                                const float degree_inc,
                                Eigen::Matrix2f &covarance) {
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
  Eigen::Vector3d transation(extrinsic_params[3], extrinsic_params[4],
                             extrinsic_params[5]);
  float fx = fx_;
  float cx = cx_;
  float fy = fy_;
  float cy = cy_;
  Eigen::Vector3f p_l(vpnp_point.x, vpnp_point.y, vpnp_point.z);
  Eigen::Vector3f p_c = rotation.cast<float>() * p_l + transation.cast<float>();

  Eigen::Matrix2f var_camera_pixel;
  var_camera_pixel << pow(pixel_inc, 2), 0, 0, pow(pixel_inc, 2);

  Eigen::Matrix2f var_lidar_pixel;
  float range = sqrt(vpnp_point.x * vpnp_point.x + vpnp_point.y * vpnp_point.y +
                     vpnp_point.z * vpnp_point.z);
  Eigen::Vector3f direction(vpnp_point.x, vpnp_point.y, vpnp_point.z);
  direction.normalize();
  Eigen::Matrix3f direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0,
      -direction(0), -direction(1), direction(0), 0;
  float range_var = range_inc * range_inc;
  Eigen::Matrix2f direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
      pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3f base_vector1(1, 1,
                               -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3f base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<float, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
      base_vector1(2), base_vector2(2);
  Eigen::Matrix<float, 3, 2> A = range * direction_hat * N;
  Eigen::Matrix3f lidar_position_var =
      direction * range_var * direction.transpose() +
      A * direction_var * A.transpose();
  Eigen::Matrix3f lidar_position_var_camera =
      rotation.cast<float>() * lidar_position_var *
      rotation.transpose().cast<float>();
  Eigen::Matrix2f lidar_pixel_var_2d;
  Eigen::Matrix<float, 2, 3> B;
  B << fx / p_c(2), 0, fx * p_c(0) / pow(p_c(2), 2), 0, fy / p_c(2),
      fy * p_c(1) / pow(p_c(2), 2);
  var_lidar_pixel = B * lidar_position_var * B.transpose();
  covarance = var_camera_pixel + var_lidar_pixel;
}

#endif