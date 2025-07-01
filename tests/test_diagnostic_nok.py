import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from diagnostic_msgs.msg import DiagnosticArray
from cameraIntel.camera_manager_node import CameraManager

@pytest.fixture(scope='module')
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_diagnostics_error_without_cameraintel(rclpy_context):
    # Create and start only the CameraManager node
    manager_node = CameraManager()
    manager_executor = SingleThreadedExecutor()
    manager_executor.add_node(manager_node)

    # Spin the node to allow it to initialize and publish diagnostics
    start_time = time.time()
    while time.time() - start_time < 10.0:  # Allow time for initialization
        manager_executor.spin_once(timeout_sec=0.1)

    # Create a test node to subscribe to diagnostics
    class TestNode(Node):
        def __init__(self):
            super().__init__('test_diagnostics_node')
            self.msg = None
            self.subscription = self.create_subscription(
                DiagnosticArray,
                '/camera_manager/diagnostics',
                self.listener_callback,
                10
            )

        def listener_callback(self, msg):
            self.msg = msg
            self.get_logger().info("Received diagnostic message")

    # Create test node and executor
    test_node = TestNode()
    test_executor = SingleThreadedExecutor()
    test_executor.add_node(test_node)

    # Spin to receive diagnostics messages
    timeout = time.time() + 15.0  # Timeout for receiving messages
    while rclpy.ok() and time.time() < timeout and test_node.msg is None:
        manager_executor.spin_once(timeout_sec=0.1)
        test_executor.spin_once(timeout_sec=0.1)

    # Verify diagnostics message was received
    assert test_node.msg is not None, "No diagnostics message received"

    # Check diagnostics levels for error (level != 0, typically 1 for warning or 2 for error)
    levels = [status.level for status in test_node.msg.status]
    assert any(level != 0 for level in levels), f"No error diagnostics found. Levels: {levels}"

    # Optionally, check for specific error level (e.g., 1 for warning or 2 for error)
    assert any(level in [b'\x01', b'\x02'] for level in levels), f"No warning (1) or error (2) diagnostics found. Levels: {levels}"

    # Clean up
    manager_executor.remove_node(manager_node)
    test_executor.remove_node(test_node)
    manager_node.destroy_node()
    test_node.destroy_node()
    manager_executor.shutdown()
    test_executor.shutdown()