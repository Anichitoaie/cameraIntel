#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import SetParametersResult
from rclpy.parameter import Parameter

class CameraIntelNode(Node):
    def __init__(self):
        super().__init__("dummy_cameraIntel_node")
        self.bridge = CvBridge()
        self.declare_parameter('fps', 30.0)
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        if self.fps <= 0:
            self.get_logger().error(f"Invalid FPS: {self.fps}. Setting to 10.0")
            self.fps = 10.0
        self.get_logger().info(f"Parameter 'fps' set to: {self.fps}")
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.pub_ir = self.create_publisher(Image, '/camera/ir/image_raw', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self._create_timer()
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(f"Initialized with FPS: {self.fps}")

    def _create_timer(self):
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.destroy()
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def parameter_callback(self, parameters):
        for param in parameters:
            if param.name == 'fps':
                new_fps = param.value
                if new_fps <= 0:
                    self.get_logger().error(f"Invalid FPS: {new_fps}. Keeping current FPS: {self.fps}")
                    return SetParametersResult(successful=False)
                self.fps = new_fps
                self._create_timer()
                self.get_logger().info(f"Updated FPS to: {self.fps}")
        return SetParametersResult(successful=True)

    def timer_callback(self):
        # Get current time for timestamp
        current_time = self.get_clock().now().to_msg()
        
        # Simulate a color frame (red with text)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (0, 0, 255)
        cv2.putText(frame, 'Simulated', (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        frame_ir = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_depth = cv2.Canny(frame, 100, 200)
        
        # Convert to ROS messages
        msg_color = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg_ir = self.bridge.cv2_to_imgmsg(frame_ir, "mono8")
        msg_depth = self.bridge.cv2_to_imgmsg(frame_depth, "mono8")
        
        # Set FPS and timestamp
        msg_color.header.frame_id = f"fps:{self.fps}"
        msg_ir.header.frame_id = f"fps:{self.fps}"
        msg_depth.header.frame_id = f"fps:{self.fps}"
        msg_color.header.stamp = current_time
        msg_ir.header.stamp = current_time
        msg_depth.header.stamp = current_time
        
        # Log timestamp for debugging
        self.get_logger().info(f"Publishing frame with FPS: {self.fps}, timestamp: {current_time.sec}.{current_time.nanosec}")
        
        # Publish messages
        self.pub_color.publish(msg_color)
        self.pub_ir.publish(msg_ir)
        self.pub_depth.publish(msg_depth)

def main(args=None):
    rclpy.init(args=args)
    node = CameraIntelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()