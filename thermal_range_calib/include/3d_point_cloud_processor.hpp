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

class PointCloudProcessor
{
private:
  /* data */
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_point_cloud_;
  std::vector<int> plane_line_number_;
  int line_number_ = 0;
  // 存储RGB图像边缘点的2D点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr rgb_edge_cloud_;
  // 存储LiDAR Depth/Intensity图像边缘点的2D点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_edge_cloud_;



  ///@todo 2023.11.21
  // read config from config file

  // point cloud edge config
  // Voxel.size: 0.5
  float voxel_size_ = 0.5;

  // Voxel.down_sample_size: 0.02

  // Plane.min_points_size: 30
  float plane_min_point_size_ = 5;  // plane_size_threshold origin

  // Plane.max_size: 8
  int plane_max_size_ = 15;  //这个参数是什么意思？

  // Ransac.dis_threshold: 0.02
  float ransac_dis_threshold_ = 0.02;

  // Edge.min_dis_threshold: 0.03
  float min_line_dis_threshold_ = 0.03;

  // Edge.max_dis_threshold: 0.06
  float max_line_dis_threshold_ = 0.06;

  // Plane.normal_theta_min: 45
  float min_plane_normal_theta_ = cos(DEG2RAD(45));


  // Plane.normal_theta_max: 135
  float max_plane_normal_theta_ = cos(DEG2RAD(135));


  cv::Mat image_, gray_image_;
  // 存储2DImage边缘点的2D点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr image_edge_cloud_;

  // voxels
  std::unordered_map<VOXEL_LOC, Voxel *> voxel_map_;

public:
  // 存储平面交接点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_line_cloud_;

  ros::NodeHandle nh_;
  ros::Publisher color_plannar_cloud_pub_ = 
        nh_.advertise<sensor_msgs::PointCloud2>("planner_cloud", 1);
  ros::Publisher line_cloud_pub_ = 
        nh_.advertise<sensor_msgs::PointCloud2>("line_cloud", 1);

  PointCloudProcessor(const std::string pcd_file,
                      const std::string calib_config_file);

  void LoadPointCloud(const std::string &pcd_file);

  void LoadPointCloudEdgeConfig(const std::string &calib_config_file);

  void InitVoxel();

  std::vector<Plane> ExtractPlaneFromVoxel(
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_voxel);
  void CalcLineFromPlanes(const std::vector<Plane> &plane_list, 
                          const Eigen::Vector3d voxel_origin_coor,
                          std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_cloud_list);
                          
  void ExtractPointCloudEdge(pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d);

  void CalLineFromPlanes(const std::vector<Plane> &plane_list, 
                         const Eigen::Vector3d voxel_origin_coor,
                         std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_cloud_list);


  // ~PointCloudProcessor();
};

PointCloudProcessor::PointCloudProcessor(const std::string pcd_file,
                                         const std::string calib_config_file)
{
  LoadPointCloud(pcd_file);
  LoadPointCloudEdgeConfig(calib_config_file);
  InitVoxel();
  ExtractPointCloudEdge(plane_line_cloud_);
  std::cout << "after ExtractPointCloudEdge() OOAD" << std::endl; 
  std::cout << "plane_line_cloud_.size()" << plane_line_cloud_->points.size() << std::endl;

}

void PointCloudProcessor::LoadPointCloud(const std::string &pcd_file){
  raw_point_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  ROS_INFO_STREAM("Loading point cloud from pcd file.");
  if (!pcl::io::loadPCDFile(pcd_file, *raw_point_cloud_)) {
    // down_sampling_voxel(*raw_lidar_cloud_, 0.02);
    std::string msg = "Sucessfully load pcd, pointcloud size: " +
                      std::to_string(raw_point_cloud_->size());
    // ywy
    ROS_INFO_STREAM(msg.c_str());
    // YWY cancel comment =====================
    
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*raw_point_cloud_, minPt, maxPt);
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
}

void PointCloudProcessor::LoadPointCloudEdgeConfig(const std::string &calib_config_file){
  // camera_intrind edge config
  // Voxel.size: 0.5
  // Voxel.down_sample_size: 0.02
  // Plane.min_points_size: 30
  // Plane.normal_theta_min: 45
  // Plane.normal_theta_max: 135
  // Plane.max_size: 8
  // Ransac.dis_threshold: 0.02
  // Edge.min_dis_threshold: 0.03
  // Edge.max_dsics_.k3_ = dist_coeffs[4];
}

void PointCloudProcessor::InitVoxel(){
  ROS_INFO_STREAM("Building Voxel");
  // for voxel test
  srand((unsigned)time(NULL));
  pcl::PointCloud<pcl::PointXYZRGB> test_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> ywy_color_voxel_cloud;
  int ywy_neg_coor_cnt = 0;
  
  for (size_t i = 0; i < raw_point_cloud_->size(); i++) {
    const pcl::PointXYZI &p_c = raw_point_cloud_->points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      // which voxel should a point be ---ywy
      loc_xyz[j] = p_c.data[j] / voxel_size_;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
        ywy_neg_coor_cnt ++;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) {
      voxel_map_[position]->cloud->push_back(p_c);
      pcl::PointXYZRGB p_rgb;
      p_rgb.x = p_c.x;
      p_rgb.y = p_c.y;
      p_rgb.z = p_c.z;
      p_rgb.r = voxel_map_[position]->voxel_color(0);
      p_rgb.g = voxel_map_[position]->voxel_color(1);
      p_rgb.b = voxel_map_[position]->voxel_color(2);
      test_cloud.push_back(p_rgb);
      // ywy_color_voxel_cloud.push_back(p_rgb);
    } else {
      Voxel *voxel = new Voxel(voxel_size_);
      voxel_map_[position] = voxel;
      voxel_map_[position]->voxel_origin_coor[0] = position.x * voxel_size_;
      voxel_map_[position]->voxel_origin_coor[1] = position.y * voxel_size_;
      voxel_map_[position]->voxel_origin_coor[2] = position.z * voxel_size_;
      voxel_map_[position]->cloud->push_back(p_c);
      int r = rand() % 256;
      int g = rand() % 256;
      int b = rand() % 256;
      voxel_map_[position]->voxel_color << r, g, b;
    }
  }
  std::cout << "ywy_neg_coor_cnt is: " << ywy_neg_coor_cnt << std::endl;

  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++) {
    if (iter->second->cloud->size() > 20) {
      down_sampling_voxel(*(iter->second->cloud), 0.02);
    }
    // loop.sleep();
  }
  std::cout << "we got " << voxel_map_.size() << " voxels" << std::endl;
  
}

std::vector<Plane> PointCloudProcessor::ExtractPlaneFromVoxel(
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_voxel){
  std::vector<Plane> plane_list;
  // 创建一个体素滤波器
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_voxel(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::copyPointCloud(*iter->second->cloud, *cloud_in_voxel);
  //创建一个模型参数对象，用于记录结果
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // inliers表示误差能容忍的点，记录点云序号
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  //创建一个分割器
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
  seg.setOptimizeCoefficients(true);
  // Mandatory-设置目标几何形状
  seg.setModelType(pcl::SACMODEL_PLANE);
  //分割方法：随机采样法
  seg.setMethodType(pcl::SAC_RANSAC);
  //设置误差容忍范围，也就是阈值
  seg.setDistanceThreshold(ransac_dis_threshold_);

  pcl::PointCloud<pcl::PointXYZRGB> color_planner_cloud;
  int plane_index = 0;
  while (cloud_in_voxel->points.size() > 10) {
    pcl::PointCloud<pcl::PointXYZI> planner_cloud;
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    //输入点云
    seg.setInputCloud(cloud_in_voxel);
    seg.setMaxIterations(500);
    //分割点云
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      ROS_INFO_STREAM(
          "Could not estimate a planner model for the given dataset");
      break;
    }
    extract.setIndices(inliers);
    extract.setInputCloud(cloud_in_voxel);
    extract.filter(planner_cloud);

    if (planner_cloud.size() > plane_min_point_size_) {
      pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
      std::vector<unsigned int> colors;
      colors.push_back(static_cast<unsigned int>(rand() % 256));
      colors.push_back(static_cast<unsigned int>(rand() % 256));
      colors.push_back(static_cast<unsigned int>(rand() % 256));
      pcl::PointXYZ p_center(0, 0, 0);
      for (size_t i = 0; i < planner_cloud.points.size(); i++) {
        pcl::PointXYZRGB p;
        p.x = planner_cloud.points[i].x;
        p.y = planner_cloud.points[i].y;
        p.z = planner_cloud.points[i].z;
        p_center.x += p.x;
        p_center.y += p.y;
        p_center.z += p.z;
        p.r = colors[0];
        p.g = colors[1];
        p.b = colors[2];
        color_cloud.push_back(p);
        color_planner_cloud.push_back(p);
      }
      p_center.x = p_center.x / planner_cloud.size();
      p_center.y = p_center.y / planner_cloud.size();
      p_center.z = p_center.z / planner_cloud.size();
      Plane single_plane;
      single_plane.cloud = planner_cloud;
      single_plane.p_center = p_center;
      // std::cout << "YWY plane center: x=" << p_center.x << " y="
      //           << p_center.y << " z="<< p_center.z << std::endl;
      single_plane.normal << coefficients->values[0],
          coefficients->values[1], coefficients->values[2];
      single_plane.index = plane_index;
      plane_list.push_back(single_plane);
      plane_index++;
    }
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZI> cloud_f;
    extract.filter(cloud_f);
    *cloud_in_voxel = cloud_f;
  }

  if (plane_list.size() >= 2) {
    ros::Rate pub_loop(5000);
    sensor_msgs::PointCloud2 planner_cloud2;
    pcl::toROSMsg(color_planner_cloud, planner_cloud2);
    planner_cloud2.header.frame_id = "world";
    color_plannar_cloud_pub_.publish(planner_cloud2);
    // std::cout << "YWY plane published, the size is " << color_planner_cloud.size() << std::endl;
    pub_loop.sleep();
  }
  return plane_list;
}

void PointCloudProcessor::CalcLineFromPlanes(
    const std::vector<Plane> &plane_list, const Eigen::Vector3d voxel_origin_coor,
    std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_cloud_list) {
  // std::cout << "PointCloudProcessor::CalcLineFromPlanes() called " << std::endl;
  if (plane_list.size() >= 2 && plane_list.size() <= plane_max_size_) {
    pcl::PointCloud<pcl::PointXYZI> temp_line_cloud;
    for (size_t plane_index1 = 0; plane_index1 < plane_list.size() - 1;
         plane_index1++) {
      for (size_t plane_index2 = plane_index1 + 1;
           plane_index2 < plane_list.size(); plane_index2++) {
        float a1 = plane_list[plane_index1].normal[0];
        float b1 = plane_list[plane_index1].normal[1];
        float c1 = plane_list[plane_index1].normal[2];
        float x1 = plane_list[plane_index1].p_center.x;
        float y1 = plane_list[plane_index1].p_center.y;
        float z1 = plane_list[plane_index1].p_center.z;
        float a2 = plane_list[plane_index2].normal[0];
        float b2 = plane_list[plane_index2].normal[1];
        float c2 = plane_list[plane_index2].normal[2];
        float x2 = plane_list[plane_index2].p_center.x;
        float y2 = plane_list[plane_index2].p_center.y;
        float z2 = plane_list[plane_index2].p_center.z;
        float theta = a1 * a2 + b1 * b2 + c1 * c2;
        //
        float point_dis_threshold = 0.00;
        if (theta > max_plane_normal_theta_ && theta < min_plane_normal_theta_) {
          // for (int i = 0; i < 6; i++) {
          if (plane_list[plane_index1].cloud.size() > 0 &&
              plane_list[plane_index2].cloud.size() > 0) {
            float matrix[4][5];
            matrix[1][1] = a1;
            matrix[1][2] = b1;
            matrix[1][3] = c1;
            matrix[1][4] = a1 * x1 + b1 * y1 + c1 * z1;
            matrix[2][1] = a2;
            matrix[2][2] = b2;
            matrix[2][3] = c2;
            matrix[2][4] = a2 * x2 + b2 * y2 + c2 * z2;
            // six types
            std::vector<Eigen::Vector3d> points;
            Eigen::Vector3d point;
            matrix[3][1] = 1;
            matrix[3][2] = 0;
            matrix[3][3] = 0;
            matrix[3][4] = voxel_origin_coor[0];
            calc<float>(matrix, point);
            if (point[0] >= voxel_origin_coor[0] - point_dis_threshold &&
                point[0] <= voxel_origin_coor[0] + voxel_size_ + point_dis_threshold &&
                point[1] >= voxel_origin_coor[1] - point_dis_threshold &&
                point[1] <= voxel_origin_coor[1] + voxel_size_ + point_dis_threshold &&
                point[2] >= voxel_origin_coor[2] - point_dis_threshold &&
                point[2] <= voxel_origin_coor[2] + voxel_size_ + point_dis_threshold) {
              points.push_back(point);
            }
            matrix[3][1] = 0;
            matrix[3][2] = 1;
            matrix[3][3] = 0;
            matrix[3][4] = voxel_origin_coor[1];
            calc<float>(matrix, point);
            if (point[0] >= voxel_origin_coor[0] - point_dis_threshold &&
                point[0] <= voxel_origin_coor[0] + voxel_size_ + point_dis_threshold &&
                point[1] >= voxel_origin_coor[1] - point_dis_threshold &&
                point[1] <= voxel_origin_coor[1] + voxel_size_ + point_dis_threshold &&
                point[2] >= voxel_origin_coor[2] - point_dis_threshold &&
                point[2] <= voxel_origin_coor[2] + voxel_size_ + point_dis_threshold) {
              points.push_back(point);
            }
            matrix[3][1] = 0;
            matrix[3][2] = 0;
            matrix[3][3] = 1;
            matrix[3][4] = voxel_origin_coor[2];
            calc<float>(matrix, point);
            if (point[0] >= voxel_origin_coor[0] - point_dis_threshold &&
                point[0] <= voxel_origin_coor[0] + voxel_size_ + point_dis_threshold &&
                point[1] >= voxel_origin_coor[1] - point_dis_threshold &&
                point[1] <= voxel_origin_coor[1] + voxel_size_ + point_dis_threshold &&
                point[2] >= voxel_origin_coor[2] - point_dis_threshold &&
                point[2] <= voxel_origin_coor[2] + voxel_size_ + point_dis_threshold) {
              points.push_back(point);
            }
            matrix[3][1] = 1;
            matrix[3][2] = 0;
            matrix[3][3] = 0;
            matrix[3][4] = voxel_origin_coor[0] + voxel_size_;
            calc<float>(matrix, point);
            if (point[0] >= voxel_origin_coor[0] - point_dis_threshold &&
                point[0] <= voxel_origin_coor[0] + voxel_size_ + point_dis_threshold &&
                point[1] >= voxel_origin_coor[1] - point_dis_threshold &&
                point[1] <= voxel_origin_coor[1] + voxel_size_ + point_dis_threshold &&
                point[2] >= voxel_origin_coor[2] - point_dis_threshold &&
                point[2] <= voxel_origin_coor[2] + voxel_size_ + point_dis_threshold) {
              points.push_back(point);
            }
            matrix[3][1] = 0;
            matrix[3][2] = 1;
            matrix[3][3] = 0;
            matrix[3][4] = voxel_origin_coor[1] + voxel_size_;
            calc<float>(matrix, point);
            if (point[0] >= voxel_origin_coor[0] - point_dis_threshold &&
                point[0] <= voxel_origin_coor[0] + voxel_size_ + point_dis_threshold &&
                point[1] >= voxel_origin_coor[1] - point_dis_threshold &&
                point[1] <= voxel_origin_coor[1] + voxel_size_ + point_dis_threshold &&
                point[2] >= voxel_origin_coor[2] - point_dis_threshold &&
                point[2] <= voxel_origin_coor[2] + voxel_size_ + point_dis_threshold) {
              points.push_back(point);
            }
            matrix[3][1] = 0;
            matrix[3][2] = 0;
            matrix[3][3] = 1;
            matrix[3][4] = voxel_origin_coor[2] + voxel_size_;
            calc<float>(matrix, point);
            if (point[0] >= voxel_origin_coor[0] - point_dis_threshold &&
                point[0] <= voxel_origin_coor[0] + voxel_size_ + point_dis_threshold &&
                point[1] >= voxel_origin_coor[1] - point_dis_threshold &&
                point[1] <= voxel_origin_coor[1] + voxel_size_ + point_dis_threshold &&
                point[2] >= voxel_origin_coor[2] - point_dis_threshold &&
                point[2] <= voxel_origin_coor[2] + voxel_size_ + point_dis_threshold) {
              points.push_back(point);
            }
            // std::cout << "points size:" << points.size() << std::endl;
            if (points.size() == 2) {
              pcl::PointCloud<pcl::PointXYZI> line_cloud;
              pcl::PointXYZ p1(points[0][0], points[0][1], points[0][2]);
              pcl::PointXYZ p2(points[1][0], points[1][1], points[1][2]);
              float length = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                                  pow(p1.z - p2.z, 2));
              // 指定近邻个数
              int K = 1;
              // 创建两个向量，分别存放近邻的索引值、近邻的中心距
              std::vector<int> pointIdxNKNSearch1(K);
              std::vector<float> pointNKNSquaredDistance1(K);
              std::vector<int> pointIdxNKNSearch2(K);
              std::vector<float> pointNKNSquaredDistance2(K);
              pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree1(
                  new pcl::search::KdTree<pcl::PointXYZI>());
              pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree2(
                  new pcl::search::KdTree<pcl::PointXYZI>());
              kdtree1->setInputCloud(
                  plane_list[plane_index1].cloud.makeShared());
              kdtree2->setInputCloud(
                  plane_list[plane_index2].cloud.makeShared());
              for (float inc = 0; inc <= length; inc += 0.005) {
                pcl::PointXYZI p;
                p.x = p1.x + (p2.x - p1.x) * inc / length;
                p.y = p1.y + (p2.y - p1.y) * inc / length;
                p.z = p1.z + (p2.z - p1.z) * inc / length;
                p.intensity = 100;
                if ((kdtree1->nearestKSearch(p, K, pointIdxNKNSearch1,
                                             pointNKNSquaredDistance1) > 0) &&
                    (kdtree2->nearestKSearch(p, K, pointIdxNKNSearch2,
                                             pointNKNSquaredDistance2) > 0)) {
                  float dis1 =
                      pow(p.x - plane_list[plane_index1]
                                    .cloud.points[pointIdxNKNSearch1[0]]
                                    .x,
                          2) +
                      pow(p.y - plane_list[plane_index1]
                                    .cloud.points[pointIdxNKNSearch1[0]]
                                    .y,
                          2) +
                      pow(p.z - plane_list[plane_index1]
                                    .cloud.points[pointIdxNKNSearch1[0]]
                                    .z,
                          2);
                  float dis2 =
                      pow(p.x - plane_list[plane_index2]
                                    .cloud.points[pointIdxNKNSearch2[0]]
                                    .x,
                          2) +
                      pow(p.y - plane_list[plane_index2]
                                    .cloud.points[pointIdxNKNSearch2[0]]
                                    .y,
                          2) +
                      pow(p.z - plane_list[plane_index2]
                                    .cloud.points[pointIdxNKNSearch2[0]]
                                    .z,
                          2);
                  if ((dis1 <
                           min_line_dis_threshold_ * min_line_dis_threshold_ &&
                       dis2 <
                           max_line_dis_threshold_ * max_line_dis_threshold_) ||
                      ((dis1 <
                            max_line_dis_threshold_ * max_line_dis_threshold_ &&
                        dis2 < min_line_dis_threshold_ *
                                   min_line_dis_threshold_))) {
                    line_cloud.push_back(p);
                  }
                }
              }
              if (line_cloud.size() > 10) {
                line_cloud_list.push_back(line_cloud);
              }
            }
          }
        }
      }
    }
  }
}




void PointCloudProcessor::ExtractPointCloudEdge(
              pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d) {
  ROS_INFO_STREAM("Extracting Lidar Edge");
  ros::Rate loop(5000);
  lidar_line_cloud_3d =
    pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);


  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++) {

    // if (iter->second->cloud->size() > 50) {
    // ywy change this cloud size to 10, origin size > 50
    if (iter->second->cloud->size() > 10) {
      std::vector<Plane> plane_list = ExtractPlaneFromVoxel(iter->second->cloud);
      
      // std::cout << "YWY Plane number in voxel OOAD: " << plane_list.size() << std::endl;

      std::vector<pcl::PointCloud<pcl::PointXYZI>> line_cloud_list;
      CalcLineFromPlanes(plane_list, iter->second->voxel_origin_coor,
                         line_cloud_list);
      // ouster 5,normal 3
      if (line_cloud_list.size() > 0 && line_cloud_list.size() <= 8) {

        for (size_t cloud_index = 0; cloud_index < line_cloud_list.size();
             cloud_index++) {
          for (size_t i = 0; i < line_cloud_list[cloud_index].size(); i++) {
            pcl::PointXYZI p = line_cloud_list[cloud_index].points[i];
            lidar_line_cloud_3d->points.push_back(p);
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


