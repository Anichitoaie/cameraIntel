import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cameraIntel.cameraIntel_node import CameraIntelNode

@pytest.fixture(scope='module')
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_color_stream_latency(rclpy_context):
    # Create node
    camera_node = CameraIntelNode()

    # Set up executor
    camera_executor = SingleThreadedExecutor()
    camera_executor.add_node(camera_node)

    # Test node for color stream
    class TestNode(Node):
        def __init__(self):
            super().__init__('test_latency_node')
            self.color_latency = None
            self.color_sub = self.create_subscription(
                Image, '/camera/color/image_raw', self.color_callback, 10)

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
    timeout = time.time() + 10.0
    while rclpy.ok() and time.time() < timeout and test_node.color_latency is None:
        camera_executor.spin_once(timeout_sec=0.1)
        test_executor.spin_once(timeout_sec=0.1)

    # Verify latency
    assert test_node.color_latency is not None, "No color stream message received"
    max_latency_ms = 100.0  # Adjust threshold as needed
    assert test_node.color_latency < max_latency_ms, (
        f"Color stream latency {test_node.color_latency} ms exceeds {max_latency_ms} ms"
    )

    # Clean up
    camera_executor.remove_node(camera_node)
    test_executor.remove_node(test_node)
    camera_node.destroy_node()
    test_node.destroy_node()
    camera_executor.shutdown()
    test_executor.shutdown()