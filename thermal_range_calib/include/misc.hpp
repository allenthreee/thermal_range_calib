// created by YWY, Nov 2023
#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>

#include <pcl/common/common.h>


enum MiscProjectionType { misc_DEPTH, misc_INTENSITY, misc_BOTH };
enum MiscDirection { misc_UP, misc_DOWN, misc_LEFT, misc_RIGHT };



void projectPointCloud2Img(const Vector6d &extrinsic_params,
                const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud,
                const MiscProjectionType projection_type, const bool is_fill_img,
                cv::Mat &projection_img, 
                double fx, double fy, double cx, double cy,
                double k1, double k2, double k3, double p1, double p2, int width, int height);

cv::Mat ywy_fillImg(const cv::Mat &input_img,
                             const MiscDirection first_direct,
                             const MiscDirection second_direct);


void projectPointCloud2Img(
    const Vector6d &extrinsic_params,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud,
    const MiscProjectionType projection_type, const bool is_fill_img,
    cv::Mat &projection_img,
    double fx, double fy, double cx, double cy,
    double k1, double k2, double k3, double p1, double p2,
    int width, int height) {
  std::vector<cv::Point3f> pts_3d;
  std::vector<float> intensity_list;
  Eigen::AngleAxisd rotation_vector3;
  rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
  int ywy_count_inrange_ponits = 0;
  for (size_t i = 0; i < lidar_cloud->size(); i++) {
    pcl::PointXYZI point_3d = lidar_cloud->points[i];
    float depth =
        sqrt(pow(point_3d.x, 2) + pow(point_3d.y, 2) + pow(point_3d.z, 2));
    // if (depth > min_depth_ && depth < max_depth_) {
    if (depth > 0) {
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
      intensity_list.emplace_back(lidar_cloud->points[i].intensity);
      ywy_count_inrange_ponits++;
    }
  }
  std::cout << "ywy point inside the depth range: " << ywy_count_inrange_ponits << std::endl;
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  cv::Mat distortion_coeff =
      (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
  cv::Mat r_vec =
      (cv::Mat_<double>(3, 1)
           << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
  cv::Mat t_vec = (cv::Mat_<double>(3, 1) << extrinsic_params[3],
                   extrinsic_params[4], extrinsic_params[5]);
  // project 3d-points into image view
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                    pts_2d);
  cv::Mat image_project = cv::Mat::zeros(height, width, CV_16UC1);
  cv::Mat rgb_image_project = cv::Mat::zeros(height, width, CV_8UC3);
  int ywy_big_intensity = 0;
  int ywy_small_intensity = 0;
  for (size_t i = 0; i < pts_2d.size(); ++i) {
    cv::Point2f point_2d = pts_2d[i];
    if (point_2d.x <= 0 || point_2d.x >= width || point_2d.y <= 0 ||
        point_2d.y >= height) {
      continue;
    } else {
      // test depth and intensity both
      if (projection_type == misc_DEPTH) {
        float depth = sqrt(pow(pts_3d[i].x, 2) + pow(pts_3d[i].y, 2) +
                           pow(pts_3d[i].z, 2));
        float intensity = intensity_list[i];
        float depth_weight = 1;
        float grey = 60000; // ywy
        // float grey = depth_weight * depth / max_depth_ * 65535 +
        //              (1 - depth_weight) * intensity / 150 * 65535;
        // 1st if-> no points has been projected to this pixel before
        if (image_project.at<ushort>(point_2d.y, point_2d.x) == 0) {
          image_project.at<ushort>(point_2d.y, point_2d.x) = grey;
          // rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] =
          //     depth / max_depth_ * 255;
          // rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] =
          //     intensity / 150 * 255;
        // 2nd if-> there is already one point here
        } else if (depth < image_project.at<ushort>(point_2d.y, point_2d.x)) {
          image_project.at<ushort>(point_2d.y, point_2d.x) = grey;
          // rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] =
          //     depth / max_depth_ * 255;
          // rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] =
          //     intensity / 150 * 255;
        }
      } else {
        // SO THIS INTENSITY IS SPECIFICALLY FOR LIVOX AVIA LIKE LIDAR
        // THAT IS DENSE LIKE A CAMERA
        // THE REPROJECTION IMAGE WILL BE ACTUALLY LIKE A IMAGE
        // std::cout << "projection_type->INTENSITY" << std::endl;
        float intensity = intensity_list[i];
        if (intensity > 100) {
          ywy_big_intensity++;
          intensity = 65535;
        } else {
          ywy_small_intensity++;
          intensity = (intensity / 150.0) * 65535;
        }
        image_project.at<ushort>(point_2d.y, point_2d.x) = intensity;
      }
    }
  }
  std::cout << "ywy_big_intensity" << ywy_big_intensity << std::endl; 
  std::cout << "ywy_small_intensity" << ywy_small_intensity << std::endl; 
  cv::Mat grey_image_projection;
  
  cv::cvtColor(rgb_image_project, grey_image_projection, cv::COLOR_BGR2GRAY);

  image_project.convertTo(image_project, CV_8UC1, 1 / 256.0);
  if (is_fill_img) {
    for (int i = 0; i < 5; i++) {
      image_project = ywy_fillImg(image_project, misc_UP, misc_LEFT);
    }
  }
  if (is_fill_img) {
    for (int i = 0; i < 5; i++) {
      grey_image_projection = ywy_fillImg(grey_image_projection, misc_UP, misc_LEFT);
    }
  }
  projection_img = image_project.clone();
}



cv::Mat ywy_fillImg(const cv::Mat &input_img,
                             const MiscDirection first_direct,
                             const MiscDirection second_direct) {
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