import numpy as np
import rospy
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler


def intersection_line(plane1, plane2):
    # Unpack the plane parameters. 
    # Plane is represented as a point and a normal vector.
    point1, normal1 = plane1
    point2, normal2 = plane2

    # The direction of the line of intersection is the cross product of the plane normals
    direction = np.cross(normal1, normal2)

    # If planes are parallel (i.e., the direction vector has zero length), they do not intersect
    if np.linalg.norm(direction) < 1e-6:
        return None

    # If the direction of the line of intersection is parallel to the XY plane
    if direction[2] != 0:
        # Use two equations derived from the plane equations, and a third equation that specifies the z-coordinate of the line
        matrix = np.array([normal1, normal2, [0, 0, 1]])
        rhs = np.array([np.dot(normal1, point1), np.dot(normal2, point2), point1[2]])
    else:
        # Use the original system of equations
        matrix = np.array([normal1, normal2, direction])
        rhs = np.array([np.dot(normal1, point1), np.dot(normal2, point2), np.dot(direction, point1)])

    # Solve the system of equations to find a point on the intersection line
    point = np.linalg.solve(matrix, rhs)
    print(f"the point is {point}")

    # Return the direction vector and a point on the line
    return direction, point


def visualize_line(direction, point):
    # rospy.init_node('line_visualizer', anonymous=True)
    # marker_pub = rospy.Publisher('visualization_line_marker', Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lines"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    # Set the scale of the marker
    marker.scale.x = 0.1  # Line width

    # Set the color
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Define the start and end points of the line
    start_point = Point()
    start_point.x = point[0] - 10 * direction[0]
    start_point.y = point[1] - 10 * direction[1]
    start_point.z = point[2] - 10 * direction[2]

    end_point = Point()
    end_point.x = point[0] + 10 * direction[0]
    end_point.y = point[1] + 10 * direction[1]
    end_point.z = point[2] + 10 * direction[2]

    marker.points.append(start_point)
    marker.points.append(end_point)

    # marker_pub.publish(marker)
    return marker

def visualize_plane(plane, plane_id):
    yaw_direction = 0
    point, normal = plane
    # marker_pub = rospy.Publisher('visualization_plane_marker', Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "planes"
    marker.id = plane_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # Set the scale of the marker
    marker.scale.x = 10.0  # Plane size in x direction
    marker.scale.y = 10.0  # Plane size in y direction
    marker.scale.z = 0.01  # Plane thickness

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5

    # Set the position of the marker
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = point[2]

    # Calculate the quaternion for the orientation
    normal = np.array(normal)
    up = np.array([0, 0, 1])
    axis = np.cross(up, normal)
    angle = np.arccos(np.dot(up, normal) / (np.linalg.norm(up) * np.linalg.norm(normal)))
    quat = tf.transformations.quaternion_about_axis(angle, axis)
    print(f"quat is {quat}")

    # Set the orientation of the marker
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    return marker



if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('plane_visualizer')

    # Create a publisher
    # pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Set the rate of publishing
    rate = rospy.Rate(10) # 10 Hz


    # Define two planes. Each plane is represented by a point and a normal vector.
    plane1 = (np.array([1, 1, 0]), np.array([1, 0, 0]))  # Plane parallel to YZ plane
    plane2 = (np.array([0, 1, 1]), np.array([0, 0, 1]))  # Plane parallel to XY plane

    # Calculate the intersection line of the two planes
    line = intersection_line(plane1, plane2)

    
    line_pub = rospy.Publisher('line_markers', MarkerArray, queue_size=10)
    plane_pub = rospy.Publisher('plane_markers', MarkerArray, queue_size=10)
    line_marker_arr = MarkerArray()
    plane_marker_arr = MarkerArray()

    if line:
        direction, point = line
        line_1 = visualize_line(direction, point)
        line_marker_arr.markers.append(line_1)
    else:
        print("The planes do not intersect.")

    # Visualize the planes in RViz
    plane_1 = visualize_plane(plane1, 1)
    plane_2 = visualize_plane(plane2, 2)
    plane_marker_arr.markers.append(plane_1)
    plane_marker_arr.markers.append(plane_2)


    while not rospy.is_shutdown():
        line_pub.publish(line_marker_arr)
        plane_pub.publish(plane_marker_arr)

        
        rate.sleep()
