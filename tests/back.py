#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraIntelNode(Node):
    def __init__(self):
        super().__init__("dummy_cameraIntel_node")

        self.bridge = CvBridge()
        self.declare_parameter('fps',30.0)
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        # Publishers
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.pub_ir = self.create_publisher(Image, '/camera/ir/image_raw', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  
        #self.fps = 30.0

    def timer_callback(self):
        # 1. Simuleaza un frame color (ro?u cu text)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (0, 0, 255)  # imagine complet ro?ie (BGR)
        cv2.putText(frame, 'Simulated', (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

        # 2. Simuleaza IR (grayscale)
        frame_ir = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 3. Simuleaza Depth (folosind edge detection)
        frame_depth = cv2.Canny(frame, 100, 200)

        # 4. Conversie �n mesaje ROS
        msg_color = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg_ir = self.bridge.cv2_to_imgmsg(frame_ir, "mono8")
        msg_depth = self.bridge.cv2_to_imgmsg(frame_depth, "mono8")

	        # Set FPS in header.frame_id
        msg_color.header.frame_id = f"fps:{self.fps}"
        msg_ir.header.frame_id = f"fps:{self.fps}"
        msg_depth.header.frame_id = f"fps:{self.fps}"

        # 5. Publicare
        self.pub_color.publish(msg_color)
        self.pub_ir.publish(msg_ir)
        self.pub_depth.publish(msg_depth)

        self.get_logger().info("Published simulated frame")
def main(args=None):
    rclpy.init(args=args)
    node = CameraIntelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()