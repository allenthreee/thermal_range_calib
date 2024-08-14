import pclpy
from pclpy import pcl
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf

class Plane:
    def __init__(self, cloud, p_center, normal, index):
        self.cloud = cloud
        self.p_center = p_center
        self.normal = normal
        self.index = index

class LineExtractor:
    def __init__(self, voxel_map, ransac_dis_thre, plane_size_threshold):
        self.voxel_map = voxel_map
        self.ransac_dis_thre = ransac_dis_thre
        self.plane_size_threshold = plane_size_threshold

    def LiDAREdgeExtraction(self):
        print("Extracting Lidar Edge")
        lidar_line_cloud_3d = pcl.PointCloud.PointXYZ()

        for voxel in self.voxel_map.values():
            if len(voxel.cloud) > 10:
                plane_list = self.planeFittingInVoxel(voxel.cloud)
                print("Plane number: ", len(plane_list))

                line_cloud_list = self.calcLineFromVoxel(plane_list, voxel.voxel_origin)
                if 0 < len(line_cloud_list) <= 8:
                    for line_cloud in line_cloud_list:
                        for p in line_cloud:
                            lidar_line_cloud_3d.push_back(p)
        print("We got ", len(lidar_line_cloud_3d), " LiDAR edges")

    def planeFittingInVoxel(self, cloud_filter):
        plane_list = []
        seg = pcl.segmentation.SACSegmentation.PointXYZ()
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(self.ransac_dis_thre)

        while cloud_filter.size() > 10:
            inliers = pcl.PointIndices()
            coefficients = pcl.ModelCoefficients()
            seg.set_input_cloud(cloud_filter)
            seg.segment(inliers, coefficients)
            if len(inliers.indices) == 0:
                print("Could not estimate a planner model for the given dataset")
                break

            planner_cloud = pcl.PointCloud.PointXYZ()
            pcl.filters.extract_indices(cloud_filter, inliers, planner_cloud, negative=False)
            if planner_cloud.size() > self.plane_size_threshold:
                p_center = np.mean(planner_cloud.xyz, axis=0)
                single_plane = Plane(planner_cloud, p_center, coefficients.values, len(plane_list))
                plane_list.append(single_plane)

            pcl.filters.extract_indices(cloud_filter, inliers, cloud_filter, negative=True)

        return plane_list

def load_pcd(file_path):
    cloud = pcl.PointCloud.PointXYZ()
    pcl.io.PCDReader().read(file_path, cloud)
    return cloud

def visualize_planes(planes):
    rospy.init_node('plane_visualizer', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    for plane in planes:
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "plane"
        marker.id = plane.index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set the position of the plane
        marker.pose.position.x = plane.p_center[0]
        marker.pose.position.y = plane.p_center[1]
        marker.pose.position.z = plane.p_center[2]
        
        # Calculate the quaternion for the orientation
        normal = np.array(plane.normal)
        up = np.array([0, 0, 1])
        axis = np.cross(up, normal)
        angle = np.arccos(np.dot(up, normal) / (np.linalg.norm(up) * np.linalg.norm(normal)))
        q = tf.transformations.quaternion_about_axis(angle, axis)
        
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # Set the scale of the plane (adjust as needed)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.01  # Thin plane
        
        # Set the color of the plane
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        # Publish the marker
        marker_pub.publish(marker)
        rospy.sleep(0.1)

if __name__ == "__main__":
    # Load the PCD file
    file_path = "/home/allen/calib_data/datasets/cjy_single_scene_calibration/0.pcd"
    cloud = load_pcd(file_path)
    
    # Create a voxel map (dummy example, replace with actual voxel map creation)
    voxel_map = {}  # Replace with actual voxel map creation logic
    
    # Initialize the LineExtractor
    ransac_dis_thre = 0.01
    plane_size_threshold = 100
    extractor = LineExtractor(voxel_map, ransac_dis_thre, plane_size_threshold)
    
    # Extract planes
    planes = extractor.planeFittingInVoxel(cloud)
    
    # Visualize planes in RViz
    visualize_planes(planes)
