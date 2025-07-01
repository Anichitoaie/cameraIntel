import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.parameter import Parameter
from cameraIntel.cameraIntel_node import CameraIntelNode
from cameraIntel.camera_manager_node import CameraManager

@pytest.fixture(scope='module')
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def run_diagnostics_test(fps, expected_level, expected_message_substring):
    # Create nodes
    camera_node = CameraIntelNode()
    
    manager_node = CameraManager()
    camera_node.set_parameters([Parameter('fps', Parameter.Type.DOUBLE, fps)])
 # Verify parameter was set
    current_fps = camera_node.get_parameter('fps').get_parameter_value().double_value
    assert current_fps == fps, f"Expected FPS {fps}, got {current_fps}"
    # Set up executors
    camera_executor = SingleThreadedExecutor()
    manager_executor = SingleThreadedExecutor()
    camera_executor.add_node(camera_node)
    manager_executor.add_node(manager_node)

    # Test node for diagnostics
    class TestNode(Node):
        def __init__(self):
            super().__init__('test_diagnostics_node')
            self.diag_msg = None
            self.subscription = self.create_subscription(
                DiagnosticArray, '/camera_manager/diagnostics', self.diag_callback, 10)

        def diag_callback(self, msg):
            self.diag_msg = msg
            self.get_logger().info("Received diagnostic message")

    test_node = TestNode()
    test_executor = SingleThreadedExecutor()
    test_executor.add_node(test_node)

    # Spin nodes
    timeout = time.time() + 10.0
    while rclpy.ok() and time.time() < timeout and test_node.diag_msg is None:
        camera_executor.spin_once(timeout_sec=0.1)
        
        manager_executor.spin_once(timeout_sec=0.1)
        test_executor.spin_once(timeout_sec=0.1)

    # Verify diagnostics
    assert test_node.diag_msg is not None, "No diagnostics message received"

    #levels = [status.level for status in test_node.diag_msg.status]
    #assert b'\x00' in levels, f"No OK (level 0) diagnostics found. Levels: {levels}"

    for status in test_node.diag_msg.status:
        assert status.level == expected_level, (
            f"Expected level {expected_level} for {status.name}, got {status.level}"
        )


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

def test_diagnostics_fps_valid(rclpy_context):
    run_diagnostics_test(fps=30.0, expected_level=DiagnosticStatus.OK, expected_message_substring="all streams ok")

def test_diagnostics_fps_invalid(rclpy_context):
    run_diagnostics_test(fps=8.0, expected_level=DiagnosticStatus.ERROR, expected_message_substring="Missing one ...")