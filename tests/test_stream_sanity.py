import time
import subprocess
import pytest
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from datetime import datetime
from cameraIntel.cameraIntel_node import CameraIntelNode
from cameraIntel.camera_manager_node import CameraManager


LOG_FILE = 'TestResult.log'
EXPECTED_TOPICS = [
    '/camera_manager/color/image_raw',
    '/camera_manager/depth/image_raw',
    '/camera_manager/ir/image_raw',
    '/camera_manager/diagnostics'
]

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

def test_topics_listed(rclpy_context):
        # Create and start the node within the test
    node = CameraIntelNode()
    node1 = CameraManager()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor1 = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor1.add_node(node1)

    # Run the node for a short period to generate topics
    
    start_time = time.time()
    while time.time() - start_time < 5.0:
        executor.spin_once(timeout_sec=0.1)
        executor1.spin_once(timeout_sec=0.1)

    test_name = "test_topics_listed"
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
        output = result.stdout.strip().splitlines()

        for topic in EXPECTED_TOPICS:
            assert topic in output, f"Topic {topic} not found"
        
        log_result(test_name, "PASS")
    except AssertionError as e:
        log_result(test_name, "FAIL", str(e))
        raise
    # Cleanly shut down the node
    executor.remove_node(node)
    executor1.remove_node(node1)
    node.destroy_node()
    node1.destroy_node()


