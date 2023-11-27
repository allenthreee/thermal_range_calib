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

#include "2d_image_processor.hpp"
#include "3d_point_cloud_processor.hpp"
#include "misc.hpp"


#define calib
#define online
class Calibration {
public:
  ros::NodeHandle nh_;

  enum ProjectionType { DEPTH, INTENSITY, BOTH };
  enum Direction { UP, DOWN, LEFT, RIGHT };

  Eigen::Vector3d adjust_euler_angle_;
  Calibration(const std::string &image_file, const std::string &pcd_file,
              const std::string &calib_config_file);

  bool loadCalibConfig(const std::string &config_file);
  bool loadConfig(const std::string &configFile);
  bool checkFov(const cv::Point2d &p);

  cv::Mat fillImg(const cv::Mat &input_img, const Direction first_direct,
                  const Direction second_direct);

  void buildPnL(const Vector6d &extrinsic_params, const int dis_threshold,
            const bool show_residual,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cam_edge_cloud_2d,
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d,
            std::vector<pnl_data> &pnp_list);

  
  void calcDirection(const std::vector<Eigen::Vector2d> &points,
                     Eigen::Vector2d &direction);
  void calcResidual(const Vector6d &extrinsic_params,
                    const std::vector<pnl_data> pnl_list,
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


  float theta_min_;
  float theta_max_;
  float direction_theta_min_;
  float direction_theta_max_;
  // float min_line_dis_threshold_ = 0.03;
  // float max_line_dis_threshold_ = 0.06;

  // 初始旋转矩阵
  Eigen::Matrix3d init_rotation_matrix_;
  // 初始平移向量
  Eigen::Vector3d init_translation_vector_;

};

Calibration::Calibration(const std::string &image_file,
                         const std::string &pcd_file,
                         const std::string &calib_config_file) {

  loadCalibConfig(calib_config_file);

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
  theta_min_ = fSettings["Plane.normal_theta_min"];
  theta_max_ = fSettings["Plane.normal_theta_max"];
  theta_min_ = cos(DEG2RAD(theta_min_));
  theta_max_ = cos(DEG2RAD(theta_max_));
  direction_theta_min_ = cos(DEG2RAD(30.0));
  direction_theta_max_ = cos(DEG2RAD(150.0));
  // color_intensity_threshold_ = fSettings["Color.intensity_threshold"];
  return true;
};



bool Calibration::checkFov(const cv::Point2d &p) {
  if (p.x > 0 && p.x < width_ && p.y > 0 && p.y < height_) {
    return true;
  } else {
    return false;
  }
}



void Calibration::buildPnL(
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
        // line_edge_cloud_2d_number.push_back(plane_line_number_[i]);
        img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
      } else {
        img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
      }
    }
  }
  if (show_residual) {

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
          // lidar_2d_number.push_back(line_edge_cloud_2d_number[i]);
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
      // pnp.number = lidar_2d_number[i];
      float theta = pnp.direction.dot(pnp.direction_lidar);
      if (theta > direction_theta_min_ || theta < direction_theta_max_) {
        pnp_list.push_back(pnp);
      }
    }
  }
  cout << "buildPnL finished" << endl;
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


void Calibration::calcResidual(const Vector6d &extrinsic_params,
                               const std::vector<pnl_data> pnl_list,
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
  for (size_t i = 0; i < pnl_list.size(); i++) {
    Eigen::Vector2d residual;
    Eigen::Matrix2f var;
    calcCovarance(extrinsic_params, pnl_list[i], 1, 0.02, 0.05, var);
    pnl_data vpnp_point = pnl_list[i];
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