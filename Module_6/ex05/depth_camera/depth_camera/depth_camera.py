from threading import Timer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Lidar(Node):

    def __init__(self):
        super().__init__('lidar')
        self.publisher_cmd_vel = self.create_publisher(
            Twist, 
            '/robot/cmd_vel', 
            10)
        self.subscriber_lidar = self.create_subscription(
            Image, 
            "/depth/image",
            self.laser_scan_callback,
            10)
        self.is_obstacle = 0
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.5, self.move_callback)

    def move_callback(self):
        twist = Twist()
        twist.linear.x = 1.0 if not self.is_obstacle else 0.0
        self.publisher_cmd_vel.publish(twist)

    def laser_scan_callback(self, img):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(f"CvBridgeError: {e}")
            return

        depth_array = np.array(depth_image, dtype=np.float32)
        h, w = depth_array.shape
        print(depth_array.shape, depth_array)

        self.is_obstacle = 1 if depth_array[120][w // 2] < 3 else 0

def main():
    rclpy.init()
    lidar = Lidar()

    rclpy.spin(lidar)
    
    lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()