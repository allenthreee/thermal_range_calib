import open3d as o3d
import numpy as np

# Define the vertices of the box
vertices = np.array([
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1]
])

# Define the faces of the box (each face is a list of vertex indices)
faces = [
    [0, 1, 2, 3],  # Bottom face
    [4, 5, 6, 7],  # Top face
    [0, 1, 5, 4],  # Front face
    [2, 3, 7, 6],  # Back face
    [0, 3, 7, 4],  # Left face
    [1, 2, 6, 5]   # Right face
]

# Function to sample points on a face
def sample_points_on_face(vertices, density_ratio):
    points = []
    for i in range(density_ratio):
        for j in range(density_ratio):
            u = i / density_ratio
            v = j / density_ratio
            point = (1 - u) * (1 - v) * vertices[0] + u * (1 - v) * vertices[1] + u * v * vertices[2] + (1 - u) * v * vertices[3]
            points.append(point)
    return points

# Sample points on all faces
density_ratio = 100  # Adjust this factor to increase/decrease density
points = []
for face in faces:
    face_vertices = vertices[face]
    points.extend(sample_points_on_face(face_vertices, density_ratio))

# Create a PointCloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))

# Save to PCD file
o3d.io.write_point_cloud("box_surface_1000.pcd", pcd)
print("we have generated the pcd file")
