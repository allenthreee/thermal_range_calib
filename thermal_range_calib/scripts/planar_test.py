import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def pointcloud_to_pointcloud2(pcd):
    points = np.asarray(pcd.points)
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        points_with_colors = np.hstack([points, colors])
    else:
        # If no color information, set colors to zeros
        colors = np.zeros_like(points)
        points_with_colors = np.hstack([points, colors])
    
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1)
    ]
    
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    return pc2.create_cloud(header, fields, points_with_colors)

def voxel_grid_to_pointcloud2(voxel_grid, pcd):
    points = np.asarray(pcd.points)
    voxel_colors = np.zeros((points.shape[0], 3))
    
    # Create a dictionary to map voxel indices to random colors
    voxel_color_map = {tuple(voxel.grid_index): np.random.rand(3) for voxel in voxel_grid.get_voxels()}
    
    # Assign random colors to points based on the voxel they belong to
    for i, point in enumerate(points):
        voxel_index = tuple((point / voxel_grid.voxel_size).astype(int))
        if voxel_index in voxel_color_map:
            voxel_colors[i] = voxel_color_map[voxel_index]
    
    points_with_colors = np.hstack([points, voxel_colors])
    
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1)
    ]
    
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    return pc2.create_cloud(header, fields, points_with_colors)

def main():
    rospy.init_node('voxel_publisher')
    voxel_pub = rospy.Publisher('/voxel_grid', PointCloud2, queue_size=10)
    raw_pub = rospy.Publisher('/raw_cloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Load and convert the PCD file to a voxel grid
    pcd = o3d.io.read_point_cloud("./corner.pcd")
    voxel_size = 0.4
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
    voxel_pc2_msg = voxel_grid_to_pointcloud2(voxel_grid, pcd)

    while not rospy.is_shutdown():
        # Publish raw point cloud
        raw_pc2_msg = pointcloud_to_pointcloud2(pcd)
        raw_pub.publish(raw_pc2_msg)

        # Publish voxel grid with random colors assigned to original points
        voxel_pub.publish(voxel_pc2_msg)

        rate.sleep()

if __name__ == '__main__':
    main()
