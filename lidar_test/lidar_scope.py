#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math

class LidarScope(Node):
    def __init__(self):
        super().__init__('lidar_scope')
        # Subscribe to the raw laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.window_name = 'RPLIDAR Scope'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 600, 600)
        
        # Scale: How many pixels per meter?
        # 80 pixels = 1 meter. So the window shows about 7 meters across.
        self.scale = 80 
        self.image_size = 600
        
        self.get_logger().info("Scope started. Waiting for laser data...")

    def scan_callback(self, msg):
        # 1. Create a black background
        img = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        
        # 2. Draw the center crosshair (The Robot)
        center_x, center_y = self.image_size // 2, self.image_size // 2
        cv2.line(img, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 1)
        cv2.line(img, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 1)

        # 3. Process the Laser Points
        angle = msg.angle_min
        for r in msg.ranges:
            # Check for valid range (infinity or 0 means bad data)
            if msg.range_min < r < msg.range_max:
                # Math: Polar to Cartesian
                # x = r * cos(theta)
                # y = r * sin(theta)
                
                # Note: We negate Y because images draw from top-to-bottom
                x = int(center_x + (r * self.scale * math.cos(angle)))
                y = int(center_y - (r * self.scale * math.sin(angle)))
                
                # Draw the point (Green)
                if 0 <= x < self.image_size and 0 <= y < self.image_size:
                    img[y, x] = (0, 255, 0)
            
            angle += msg.angle_increment

        # 4. Add Text
        cv2.putText(img, "RPLIDAR Raw Data", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # 5. Show
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarScope()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
