import numpy as np
import rospy
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from tf.transformations import quaternion_about_axis

def intersection_line(plane1, plane2):
    point1, normal1 = plane1
    point2, normal2 = plane2

    direction = np.cross(normal1, normal2)

    if np.linalg.norm(direction) < 1e-6:
        return None

    if direction[2] != 0:
        matrix = np.array([normal1, normal2, [0, 0, 1]])
        rhs = np.array([np.dot(normal1, point1), np.dot(normal2, point2), point1[2]])
    else:
        matrix = np.array([normal1, normal2, direction])
        rhs = np.array([np.dot(normal1, point1), np.dot(normal2, point2), np.dot(direction, point1)])

    point = np.linalg.solve(matrix, rhs)
    return direction, point

def visualize_line(direction, point, line_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lines"
    marker.id = line_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

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

    return marker

def visualize_plane(plane, plane_id):
    point, normal = plane
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "planes"
    marker.id = plane_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 10.0
    marker.scale.y = 10.0
    marker.scale.z = 0.01
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = point[2]

    up = np.array([0, 0, 1])
    axis = np.cross(up, normal)
    angle = np.arccos(np.dot(up, normal) / (np.linalg.norm(up) * np.linalg.norm(normal)))
    quat = quaternion_about_axis(angle, axis)

    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    return marker

if __name__ == '__main__':
    rospy.init_node('plane_visualizer')
    rate = rospy.Rate(10)

    plane1 = (np.array([4, 0, 0]), np.array([1, 0.1, 0]))  # YZ plane
    plane2 = (np.array([0, 4, 0]), np.array([0, 1, 0]))  # XZ plane
    plane3 = (np.array([0, 0, 4]), np.array([0, 0, 1]))  # XY plane

    line_pub = rospy.Publisher('line_markers', MarkerArray, queue_size=10)
    plane_pub = rospy.Publisher('plane_markers', MarkerArray, queue_size=10)
    line_marker_arr = MarkerArray()
    plane_marker_arr = MarkerArray()

    for planes in [(plane1, plane2), (plane1, plane3), (plane3, plane2)]:
        line = intersection_line(planes[0], planes[1])
        if line:
            direction, point = line
            line_id = len(line_marker_arr.markers)
            line_marker = visualize_line(direction, point, line_id)
            line_marker_arr.markers.append(line_marker)
        else:
            print("The planes do not intersect.")

    for plane, plane_id in [(plane1, 1), (plane2, 2), (plane3, 3)]:
        plane_marker = visualize_plane(plane, plane_id)
        plane_marker_arr.markers.append(plane_marker)

    while not rospy.is_shutdown():
        line_pub.publish(line_marker_arr)
        plane_pub.publish(plane_marker_arr)
        rate.sleep()