#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from autoware_auto_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Point, Quaternion, Pose

class RandomDetection3DPublisher(Node):
    def __init__(self):
        super().__init__('random_detection3d_array')
        
        # Create a publisher for the detection3d array message
        self.detection3d_array_pub = self.create_publisher(Detection3DArray, 'detection3d_array', 10)
        
        # Set the loop rate (Hz)
        self.rate = self.create_rate(10)
        
        # Start publishing the detection3d array
        self.publish_detection3d_array()

    def publish_detection3d_array(self):
        while True:
            # Generate a random detection3d array message
            detection3d_array_msg = self.random_detection3d_array(5)

            # Publish the detection3d array message
            self.detection3d_array_pub.publish(detection3d_array_msg)

            # Sleep until the next cycle
            self.rate.sleep()

    def random_detection3d_array(self, num_detections):
        # Create a new Detection3DArray message
        detection3d_array_msg = Detection3DArray()

        # Loop through the desired number of detections and create a Detection3D message for each one
        for i in range(num_detections):
            # Create a new Detection3D message
            detection3d_msg = Detection3D()

            # Set the detection's label and score
            detection3d_msg.label = np.random.randint(0, 10)
            detection3d_msg.score = np.random.rand()

            # Set the detection's pose (random position and orientation)
            detection_pose = Pose()
            detection_pose.position = Point(np.random.uniform(-10.0, 10.0), np.random.uniform(-10.0, 10.0), np.random.uniform(-10.0, 10.0))
            detection_pose.orientation = Quaternion(np.random.uniform(-1.0, 1.0), np.random.uniform(-1.0, 1.0), np.random.uniform(-1.0, 1.0), np.random.uniform(-1.0, 1.0))
            detection3d_msg.pose = detection_pose

            # Set the detection's dimensions (random width, height, and length)
            detection3d_msg.dimensions.x = np.random.uniform(0.5, 2.0)
            detection3d_msg.dimensions.y = np.random.uniform(0.5, 2.0)
            detection3d_msg.dimensions.z = np.random.uniform(0.5, 2.0)

            # Add the detection message to the array
            detection3d_array_msg.detections.append(detection3d_msg)

        return detection3d_array_msg

def main(args=None):
    rclpy.init(args=args)

    node = RandomDetection3DPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
