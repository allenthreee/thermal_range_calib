#include "include/thermal_range_calib.hpp"
#include "ceres/ceres.h"
#include "include/common.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/eigen.hpp>

using namespace std;

// Data path
string image_file;
string pcd_file;
string result_file;

// Camera config
vector<double> camera_matrix;
vector<double> dist_coeffs;
double width;
double height;

// Calib config
bool use_rough_calib;
string calib_config_file;
// instrins matrix
Eigen::Matrix3d inner;
// Distortion coefficient
Eigen::Vector4d distor;
Eigen::Vector4d quaternion;
Eigen::Vector3d transation;

// pnl calib
class PnLCalib {
public:
  PnLCalib(pnl_data p) { pd = p; }
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
    return (new ceres::AutoDiffCostFunction<PnLCalib, 2, 4, 3>(
        new PnLCalib(p)));
  }

private:
  pnl_data pd;
};

int main(int argc, char **argv) {

  // init all params
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  ros::Rate loop_rate(0.1);

  nh.param<string>("common/image_file", image_file, "");
  nh.param<string>("common/pcd_file", pcd_file, "");
  nh.param<string>("common/result_file", result_file, "");
  std::cout << "pcd_file path:" << pcd_file << std::endl;
  nh.param<vector<double>>("camera/camera_matrix", camera_matrix,
                           vector<double>());
  nh.param<vector<double>>("camera/dist_coeffs", dist_coeffs, vector<double>());
  nh.param<bool>("calib/use_rough_calib", use_rough_calib, false);
  nh.param<string>("calib/calib_config_file", calib_config_file, "");

  Calibration calibra(image_file, pcd_file, calib_config_file); // we extract lidar lines here in the construction function
  
  calibra.fx_ = camera_matrix[0];
  calibra.cx_ = camera_matrix[2];
  calibra.fy_ = camera_matrix[4];
  calibra.cy_ = camera_matrix[5];
  calibra.k1_ = dist_coeffs[0];
  calibra.k2_ = dist_coeffs[1];
  calibra.p1_ = dist_coeffs[2];
  calibra.p2_ = dist_coeffs[3];
  calibra.k3_ = dist_coeffs[4];
  Eigen::Vector3d init_euler_angle =
      calibra.init_rotation_matrix_.eulerAngles(2, 1, 0);
  Eigen::Vector3d init_transation = calibra.init_translation_vector_;

  Vector6d calib_params;
  calib_params << init_euler_angle(0), init_euler_angle(1), init_euler_angle(2),
      init_transation(0), init_transation(1), init_transation(2);

  std::vector<PnPData> pnp_list;
  std::vector<pnl_data> vpnp_list;

  ROS_INFO_STREAM("Finish prepare!");

  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  inner << calibra.fx_, 0.0, calibra.cx_, 0.0, calibra.fy_, calibra.cy_, 0.0,
      0.0, 1.0;
  distor << calibra.k1_, calibra.k2_, calibra.p1_, calibra.p2_;
  R = calibra.init_rotation_matrix_;
  T = calibra.init_translation_vector_;
  std::cout << "Initial rotation matrix:" << std::endl
            << calibra.init_rotation_matrix_ << std::endl;
  std::cout << "Initial translation:"
            << calibra.init_translation_vector_.transpose() << std::endl;
  Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
  calib_params[0] = euler[0];
  calib_params[1] = euler[1];
  calib_params[2] = euler[2];
  calib_params[3] = T[0];
  calib_params[4] = T[1];
  calib_params[5] = T[2];
  sensor_msgs::PointCloud2 pub_cloud; 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  calibra.colorCloud(calib_params, 5, calibra.image_, calibra.raw_lidar_cloud_,
                     rgb_cloud);
  pcl::toROSMsg(*rgb_cloud, pub_cloud);
  pub_cloud.header.frame_id = "livox";
  calibra.init_rgb_cloud_pub_.publish(pub_cloud);


  int iter = 0;
  // Maximum match distance threshold: 15 pixels
  // If initial extrinsic lead to error over 15 pixels, the algorithm will not
  // work
  int dis_threshold = 30;
  bool opt_flag = true;
  ros::Rate loop(0.5);
  // roughCalib(calibra, calib_params, DEG2RAD(0.01), 20);


  // cv::Mat opt_img = calibra.getProjectionImg(calib_params);
  // cv::Mat small_opt_img = cv::Mat::zeros(cv::Size(1280,720), CV_64FC1);
  // cv::resize(opt_img, small_opt_img, small_opt_img.size(), 0, 0, cv::INTER_LINEAR);
  // cv::imshow("Optimization result", small_opt_img);
  // cv::imwrite("/home/allen/calib_ws/data_demo/results/opt.png", opt_img);
  // std::cout << "extrinsic param saved in result file now :)" << std::endl;
  // cv::waitKey(1000);

  while (ros::ok()) {
    std::cout << "push enter to publish again" << std::endl;
    getchar();
    /* code */
  }
  return 0;
}