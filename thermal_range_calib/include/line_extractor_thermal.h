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


class ThermalLineExtractor{

public:
  ThermalLineExtractor(pnl_data p) { pd = p; }
  template <typename T>
  bool operator()(const T *_q, const T *_t, T *residuals) const {
    Eigen::Matrix<T, 3, 3> innerT = inner.cast<T>();
    Eigen::Matrix<T, 4, 1> distorT = distor.cast<T>();
    Eigen::Quaternion<T> q_incre{_q[3], _q[0], _q[1], _q[2]};
    Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};
    Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
    Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
    Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;
    T uo = p_2[0] / p_2[2];
    T vo = p_2[1] / p_2[2];
    const T &fx = innerT.coeffRef(0, 0);
    const T &cx = innerT.coeffRef(0, 2);
    const T &fy = innerT.coeffRef(1, 1);
    const T &cy = innerT.coeffRef(1, 2);
    T xo = (uo - cx) / fx;
    T yo = (vo - cy) / fy;
    T r2 = xo * xo + yo * yo;
    T r4 = r2 * r2;
    T distortion = 1.0 + distorT[0] * r2 + distorT[1] * r4;
    T xd = xo * distortion + (distorT[2] * xo * yo + distorT[2] * xo * yo) +
           distorT[3] * (r2 + xo * xo + xo * xo);
    T yd = yo * distortion + distorT[3] * xo * yo + distorT[3] * xo * yo +
           distorT[2] * (r2 + yo * yo + yo * yo);
    T ud = fx * xd + cx;
    T vd = fy * yd + cy;
    if (T(pd.direction(0)) == T(0.0) && T(pd.direction(1)) == T(0.0)) {
      residuals[0] = ud - T(pd.u);
      residuals[1] = vd - T(pd.v);
    } else {
      residuals[0] = ud - T(pd.u);
      residuals[1] = vd - T(pd.v);
      Eigen::Matrix<T, 2, 2> I =
          Eigen::Matrix<float, 2, 2>::Identity().cast<T>();
      Eigen::Matrix<T, 2, 1> n = pd.direction.cast<T>();
      Eigen::Matrix<T, 1, 2> nt = pd.direction.transpose().cast<T>();
      Eigen::Matrix<T, 2, 2> V = n * nt;
      V = I - V;
      Eigen::Matrix<T, 2, 1> R = Eigen::Matrix<float, 2, 1>::Zero().cast<T>();
      R.coeffRef(0, 0) = residuals[0];
      R.coeffRef(1, 0) = residuals[1];
      R = V * R;
      // Eigen::Matrix<T, 2, 2> R = Eigen::Matrix<float, 2,
      // 2>::Zero().cast<T>(); R.coeffRef(0, 0) = residuals[0];
      // R.coeffRef(1, 1) = residuals[1]; R = V * R * V.transpose();
      residuals[0] = R.coeffRef(0, 0);
      residuals[1] = R.coeffRef(1, 0);
    }
    return true;
  }
  static ceres::CostFunction *Create(pnl_data p) {
    return (new ceres::AutoDiffCostFunction<ThermalLineExtractor, 2, 4, 3>(
        new ThermalLineExtractor(p)));
  }

  // Detect edge by canny, and filter by edge length
  void ThermalLineExtractor::EdgeDetector(
    const int &canny_threshold, const int &edge_threshold,
    const cv::Mat &src_img, cv::Mat &edge_img,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_cloud) {
        int gaussian_size = 5;
        // cv::GaussianBlur(src_img, src_img, cv::Size(gaussian_size, gaussian_size), 0,
        //                  0);
        cv::Mat canny_result = cv::Mat::zeros(height_, width_, CV_8UC1);
        cv::Canny(src_img, canny_result, canny_threshold, canny_threshold * 3, 3,
                    true);
        // ywy debug
        std::cout << "ywy test canny_threshold" << canny_threshold << std::endl;
        cv::Mat canny_img = cv::Mat::zeros(cv::Size(1280,720), CV_64FC1);
        cv::resize(canny_result, canny_img, canny_img.size(), 0, 0, cv::INTER_LINEAR);
        cv::imshow("canny test", canny_img);
        // ywy debug

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
                        cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        edge_img = cv::Mat::zeros(height_, width_, CV_8UC1);

        edge_cloud =
            pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < contours.size(); i++) {
            if (contours[i].size() > edge_threshold) {
            cv::Mat debug_img = cv::Mat::zeros(height_, width_, CV_8UC1);
            for (size_t j = 0; j < contours[i].size(); j++) {
                pcl::PointXYZ p;
                p.x = contours[i][j].x;
                p.y = -contours[i][j].y;
                p.z = 0;
                edge_img.at<uchar>(-p.y, p.x) = 255;
            }
            }
        }
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
        // cv::imshow("canny result", canny_result);
        // cv::imshow("edge result", edge_img);
        // cv::waitKey();
  }

private:
  pnl_data pd;
};