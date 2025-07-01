import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cameraIntel.cameraIntel_node import CameraIntelNode
from cameraIntel.camera_manager_node import CameraManager
from rclpy.parameter import Parameter
from datetime import datetime

LOG_FILE = 'TestResult.log'

# Utility for logging
def log_result(test_name, result, message=""):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"[{timestamp}] {test_name}: {result}\n")
        if message:
            f.write(f"  Info: {message}\n")

@pytest.fixture(scope='module')
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_color_stream_latency(rclpy_context):
    test_name = "test_latency"
    # Create nodes
    camera_intel_node = CameraIntelNode()
    camera_intel_node.set_parameters([Parameter('fps', Parameter.Type.DOUBLE, 15.0)])  # Valid FPS
    camera_manager_node = CameraManager()

    # Set up executors
    camera_intel_executor = SingleThreadedExecutor()
    camera_manager_executor = SingleThreadedExecutor()
    camera_intel_executor.add_node(camera_intel_node)
    camera_manager_executor.add_node(camera_manager_node)

    # Test node for color stream
    class TestNode(Node):
        def __init__(self):
            super().__init__('test_latency_node')
            self.color_latency = None
            self.color_sub = self.create_subscription(
                Image, '/camera_manager/color/image_raw', self.color_callback, 10)

        def color_callback(self, msg):
            # Calculate latency: current time - message timestamp
            current_time = self.get_clock().now()
            msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
            latency = (current_time - msg_time).nanoseconds / 1e6  # Convert to milliseconds
            self.color_latency = latency
            self.get_logger().info(f"Color stream latency: {latency} ms")

    test_node = TestNode()
    test_executor = SingleThreadedExecutor()
    test_executor.add_node(test_node)

    # Spin nodes
    timeout = time.time() + 15.0  # Increased timeout
    while rclpy.ok() and time.time() < timeout and test_node.color_latency is None:
        camera_intel_executor.spin_once(timeout_sec=0.1)
        camera_manager_executor.spin_once(timeout_sec=0.1)
        test_executor.spin_once(timeout_sec=0.1)

    # Verify latency
    try:
        assert test_node.color_latency is not None, "No color stream message received"
        max_latency_ms = 100.0  # Adjust threshold as needed
        assert test_node.color_latency < max_latency_ms, (
            f"Color stream latency {test_node.color_latency} ms exceeds {max_latency_ms} ms"
        )
        log_result(test_name, "PASS")
    except AssertionError as e:
        log_result(test_name, "FAIL", str(e))
        raise

    # Clean up
    camera_intel_executor.remove_node(camera_intel_node)
    camera_manager_executor.remove_node(camera_manager_node)
    test_executor.remove_node(test_node)
    camera_intel_node.destroy_node()
    camera_manager_node.destroy_node()
    test_node.destroy_node()
    camera_intel_executor.shutdown()
    camera_manager_executor.shutdown()
    test_executor.shutdown()