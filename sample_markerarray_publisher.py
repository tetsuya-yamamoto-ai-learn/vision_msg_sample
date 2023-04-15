#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion, Pose

def rpy_to_quaternion(roll, pitch, yaw):
    q = np.zeros((4,))
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    q[0] = cy * cr * cp + sy * sr * sp
    q[1] = cy * sr * cp - sy * cr * sp
    q[2] = cy * cr * sp + sy * sr * cp
    q[3] = sy * cr * cp - cy * sr * sp
    return Quaternion(q[1], q[2], q[3], q[0])

class RandomMarkerArrayPublisher(Node):
    def __init__(self):
        super().__init__('random_marker_array')
        # Create a publisher for the marker array message
        self.marker_array_pub = self.create_publisher(MarkerArray, 'marker_array', 10)
        # Start publishing the marker array
        self.publish_marker_array()
        
    def publish_marker_array(self):
        while True:
            # Generate a random marker array message
            marker_array_msg = self.random_marker_array(20)
            # Publish the marker array message
            self.marker_array_pub.publish(marker_array_msg)
            # publish message
            print(f"publish marker array {type(marker_array_msg)}")
            # Sleep
            time.sleep(0.1)

    def random_marker_array(self, num_markers):
        # Create a new MarkerArray message
        marker_array_msg = MarkerArray()
        # Loop through the desired number of markers and create a Marker message for each one
        for i in range(num_markers):
            # Create a new Marker message
            marker_msg = Marker()
            # set frame id for rviz show
            marker_msg.header.frame_id = 'map'
            # Set the marker's namespace, ID, and type
            marker_msg.ns = "random_markers"
            marker_msg.id = i
            marker_msg.type = Marker.CUBE
            # Set the marker's scale
            marker_msg.scale.x = 0.1
            marker_msg.scale.y = 0.1
            marker_msg.scale.z = 0.1
            # Set the marker's color
            marker_msg.color.r = np.random.rand()
            marker_msg.color.g = np.random.rand()
            marker_msg.color.b = np.random.rand()
            marker_msg.color.a = 0.3
            # Set the marker's pose (random position and orientation)
            marker_pose = Pose()
            marker_pose.position = Point(x=np.random.uniform(-1.0, 1.0), y=np.random.uniform(-1.0, 1.0), z=np.random.uniform(-1.0, 1.0))
            marker_pose.orientation = Quaternion(x=np.random.uniform(-1.0, 1.0), y=np.random.uniform(-1.0, 1.0), z=np.random.uniform(-1.0, 1.0), w=np.random.uniform(-1.0, 1.0))
            marker_msg.pose = marker_pose

            # Add the marker message to the array
            marker_array_msg.markers.append(marker_msg)

        return marker_array_msg


def main(args=None):
    rclpy.init(args=args)

    node = RandomMarkerArrayPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
