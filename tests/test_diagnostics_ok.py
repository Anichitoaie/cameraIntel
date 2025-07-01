import time
import subprocess
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from diagnostic_msgs.msg import DiagnosticArray
from cameraIntel.cameraIntel_node import CameraIntelNode
from cameraIntel.camera_manager_node import CameraManager

EXPECTED_TOPICS = [
    '/camera/color/image_raw',
    '/camera/depth/image_raw',
    '/camera/infrared/image_raw',
    '/camera_manager/diagnostics'
]

@pytest.fixture(scope='module')
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_diagnostics_ok(rclpy_context):
    # Create and start the nodes
    camera_node = CameraIntelNode()
    manager_node = CameraManager()
    
    # Create separate executors for each node
    camera_executor = SingleThreadedExecutor()
    manager_executor = SingleThreadedExecutor()
    camera_executor.add_node(camera_node)
    manager_executor.add_node(manager_node)

    # Spin nodes to allow them to initialize and publish
    start_time = time.time()
    while time.time() - start_time < 10.0:  # Increased spin time to ensure initialization
        camera_executor.spin_once(timeout_sec=0.1)
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
    timeout = time.time() + 15.0  # Increased timeout for receiving messages
    while rclpy.ok() and time.time() < timeout and test_node.msg is None:
        camera_executor.spin_once(timeout_sec=0.1)
        manager_executor.spin_once(timeout_sec=0.1)
        test_executor.spin_once(timeout_sec=0.1)

    # Verify diagnostics message was received
    assert test_node.msg is not None, "No diagnostics message received"

    # Check diagnostics levels
    levels = [status.level for status in test_node.msg.status]
    assert b'\x00' in levels, f"No OK (level 0) diagnostics found. Levels: {levels}"

    # Clean up
    camera_executor.remove_node(camera_node)
    manager_executor.remove_node(manager_node)
    test_executor.remove_node(test_node)
    camera_node.destroy_node()
    manager_node.destroy_node()
    test_node.destroy_node()
    camera_executor.shutdown()
    manager_executor.shutdown()
    test_executor.shutdown()