import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
import numpy as np

def visualize_plane(center, normal, pub):
    # Create the Marker message
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.CUBE
    marker.action = marker.ADD

    # Set the pose of the marker to the center of the plane
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]

    # Calculate the rotation angles to align the z-axis with the normal
    roll = 0
    pitch = np.arctan2(-normal[0], np.sqrt(normal[1]**2 + normal[2]**2))
    yaw = np.arctan2(normal[1], normal[2])

    # Convert the Euler angles to a quaternion
    quat = quaternion_from_euler(roll, pitch, yaw)
    print(f"quat is {quat}")

    # Set the orientation of the marker
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    # Set the scale of the marker to represent the plane
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 0.01

    # Set the color and transparency (alpha)
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # Publish the Marker message
    pub.publish(marker)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('plane_visualizer')

    # Create a publisher
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Define the center point and normal of the plane
    center = [0, 0, 0]
    normal = [1, 0, 1]

    # Normalize the normal vector
    normal = normal / np.linalg.norm(normal)

    # Set the rate of publishing
    rate = rospy.Rate(10) # 10 Hz

    # Keep publishing the plane
    while not rospy.is_shutdown():
        visualize_plane(center, normal, pub)
        rate.sleep()