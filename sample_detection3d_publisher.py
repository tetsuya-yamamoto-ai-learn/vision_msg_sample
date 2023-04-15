import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
import random
import time

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.publisher_ = self.create_publisher(Detection3DArray, 'detections', 10)
        self.detect_objects()

    def detect_objects(self):
        while True:
            msg = Detection3DArray()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            for i in range(random.randint(1, 5)):
                detection = Detection3D()
                detection.header.frame_id = 'map'
                detection.header.stamp = msg.header.stamp
                detection.bbox.center.position.x = random.uniform(-1, 1)
                detection.bbox.center.position.y = random.uniform(-1, 1)
                detection.bbox.center.position.z = random.uniform(0, 3)
                detection.bbox.center.orientation.x = random.uniform(0, 3)
                detection.bbox.center.orientation.y = random.uniform(0, 3)
                detection.bbox.center.orientation.z = random.uniform(0, 3)
                detection.bbox.size.x = random.uniform(0.1, 0.5)
                detection.bbox.size.y = random.uniform(0.1, 0.5)
                detection.bbox.size.z = random.uniform(0.1, 0.5)
                object_id = str(int(random.uniform(0, 10)))
                detection.id = 'object_' + object_id
                hypothesis_with_pose = ObjectHypothesisWithPose()
                hypothesis_with_pose.hypothesis.class_id = 'person'
                hypothesis_with_pose.hypothesis.score = random.uniform(0, 1.0)
                detection.results.append(hypothesis_with_pose)
                msg.detections.append(detection)
            self.publisher_.publish(msg)
            print("publish")
            # sleep
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
