from threading import Timer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Lidar(Node):

    def __init__(self):
        super().__init__('lidar')
        self.publisher_cmd_vel = self.create_publisher(
            Twist, 
            '/robot/cmd_vel', 
            10)
        self.subscriber_lidar = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.laser_scan_callback,
            10)
        self.is_obstacle = 0
        self.laser_scan = LaserScan()
        self.timer = self.create_timer(0.5, self.move_callback)

    def move_callback(self):
        twist = Twist()
        twist.linear.x = 1.0 if not self.is_obstacle else 0.0
        self.publisher_cmd_vel.publish(twist)

    def laser_scan_callback(self, scan):
        scan = scan.ranges
        all_more = True
        if len(scan) != 0:
            for i in scan[int(len(scan) / 2) - 15:int(len(scan) / 2) + 15]:
                if i < 1.0:
                    all_more = False
                    break
            self.is_obstacle = 0 if all_more else 1

def main():
    rclpy.init()
    lidar = Lidar()

    rclpy.spin(lidar)
    
    lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()