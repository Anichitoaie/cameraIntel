#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time

class CameraManager(Node):
    def __init__(self):
        super().__init__("stream_republisher_node")
        self.allowed_fps =[6,15,30,60]
        # Subscriber topics
        #one more comment+111+1+11
        self.subscription_color = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10)
        self.subscription_depth = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.subscription_ir = self.create_subscription(
            Image, '/camera/ir/image_raw', self.ir_callback, 10)

        # Republisher topics
        self.publisher_color = self.create_publisher(Image, '/camera_manager/color/image_raw', 10)
        self.publisher_depth = self.create_publisher(Image, '/camera_manager/depth/image_raw', 10)
        self.publisher_ir = self.create_publisher(Image, '/camera_manager/ir/image_raw', 10)

        # Diagnostic publisher
        self.diagnostic_publisher = self.create_publisher(DiagnosticArray, '/camera_manager/diagnostics', 10)

        #Track FPS
        self.fps = {'color':None,'ir':None,'depth':None}
        # Track received messages
        self.received_color = False
        self.received_depth = False
        self.received_ir = False

        # Timer for diagnostics
        self.timer = self.create_timer(1.0, self.publish_diagnostics)  # Check every 1 second

    def color_callback(self, msg):
        self.received_color = True
        fps=self.extract_fps(msg)
        self.fps['color'] = fps
        if fps not in self.allowed_fps:
            self.received_color=False
        self.publisher_color.publish(msg)
        self.get_logger().info("Republished color image")

    def depth_callback(self, msg):
        self.received_depth = True
        fps=self.extract_fps(msg)
        self.fps['depth'] = fps
        if fps not in self.allowed_fps:
            self.received_color=False
        self.publisher_depth.publish(msg)
        self.get_logger().info("Republished depth image")

    def ir_callback(self, msg):
        self.received_ir = True
        fps=self.extract_fps(msg)
        self.fps['ir'] = fps
        if fps not in self.allowed_fps:
            self.received_color=False
        self.publisher_ir.publish(msg)
        self.get_logger().info("Republished IR image")
	
    def extract_fps(self, msg):
        try:
            if msg.header.frame_id.startswith("fps:"):
                return float(msg.header.frame_id.split(":")[1])
            return 0.0
        except (IndexError, ValueError):
            self.get_logger().error(f"Invalid FPS format in frame_id: {msg.header.frame_id}")
            return 0.0

    def publish_diagnostics(self):
        # Create DiagnosticArray message
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        # Create a single DiagnosticStatus
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if (self.received_color and self.received_depth and self.received_ir) else DiagnosticStatus.ERROR
        status.name = "stream_republisher_node/status"
        status.message = "All streams OK" if status.level == DiagnosticStatus.OK else "Missing one or more streams"
        status.values.append(KeyValue(key="color_received", value=str(self.received_color)))
        status.values.append(KeyValue(key="depth_received", value=str(self.received_depth)))
        status.values.append(KeyValue(key="ir_received", value=str(self.received_ir)))

        # Add status to array
        diag_msg.status = [status]

        # Publish the diagnostic message
        self.diagnostic_publisher.publish(diag_msg)
        self.get_logger().info(f"Published diagnostics: {status.message} (Level: {status.level})")

        # Reset flags for next cycle
        self.received_color = False
        self.received_depth = False
        self.received_ir = False

def main(args=None):
    rclpy.init(args=args)
    node = CameraManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()