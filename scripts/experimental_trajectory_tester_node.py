import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_listener import TransformListenerException
from rclpy.duration import Duration
import random
import math
import subprocess
import os
import datetime
import time  # For time.sleep


class MapAwareTrajectoryTester(Node):
    def __init__(self):
        super().__init__('map_aware_trajectory_tester')
        self.get_logger().info('Map-Aware Trajectory Tester Node Started.')

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 navigate_to_pose action server...")
        if not self._action_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Nav2 action server not available! Exiting.")
            rclpy.shutdown()
            return
        self.get_logger().info("Nav2 action server found.")

        # TF Listener for robot's current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map Subscriber
        self.current_map = None
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10  # QoS history depth
        )
        self.get_logger().info("Subscribing to /map topic.")

        # Parameters for goal generation (can be loaded from YAML)
        self.declare_parameter('min_goal_distance', 0.5)
        self.declare_parameter('max_goal_distance', 5.0)
        self.declare_parameter('clearance_buffer', 0.1)  # Extra buffer around robot_radius
        self.declare_parameter('goal_attempts_per_cycle', 100)
        self.declare_parameter('test_duration_minutes', 30)  # How long to run the test for
        self.declare_parameter('robot_radius', 0.25)  # Essential for clearance check

        self.min_goal_distance = self.get_parameter('min_goal_distance').get_parameter_value().double_value
        self.max_goal_distance = self.get_parameter('max_goal_distance').get_parameter_value().double_value
        self.clearance_buffer = self.get_parameter('clearance_buffer').get_parameter_value().double_value
        self.goal_attempts_per_cycle = self.get_parameter('goal_attempts_per_cycle').get_parameter_value().integer_value
        self.test_duration_minutes = self.get_parameter('test_duration_minutes').get_parameter_value().integer_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value

        self.rosbag_process = None
        self.bag_directory = os.path.expand_user("~/ros2_nav2_debug_bags")
        os.makedirs(self.bag_directory, exist_ok=True)

        self.start_time = self.get_clock().now()

        # Timer to periodically check for next goal
        self.timer = self.create_timer(1.0, self.timer_callback)  # Check every 1 second
        self.current_goal_handle = None
        self.get_logger().info("Waiting for first map message...")

    def map_callback(self, msg):
        # Update map only if it's new or significantly different
        if self.current_map is None or msg.header.stamp.sec > self.current_map.header.stamp.sec:
            self.current_map = msg
            self.get_logger().info(f"Received new map with resolution {self.current_map.info.resolution}m.")

    def get_robot_pose(self):
        try:
            # Lookup the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except TransformListenerException as ex:
            self.get_logger().warn(f"Could not transform map to base_link: {ex}")
            return None

    def is_pose_clear(self, x, y, map_data, map_info, buffer_radius):
        """Checks if a circular area around (x,y) is clear in the map."""
        map_res = map_info.resolution
        map_origin_x = map_info.origin.position.x
        map_origin_y = map_info.origin.position.y
        map_width = map_info.width
        map_height = map_info.height

        # Convert world coords to map pixels
        px = int((x - map_origin_x) / map_res)
        py = int((y - map_origin_y) / map_res)

        # Check a square bounding box around the circle for simplicity
        buffer_pixels = int(buffer_radius / map_res)

        for i in range(px - buffer_pixels, px + buffer_pixels + 1):
            for j in range(py - buffer_pixels, py + buffer_pixels + 1):
                # Check bounds
                if 0 <= i < map_width and 0 <= j < map_height:
                    # Check if pixel is within the circular buffer_radius
                    dist_sq = ((i - px) * map_res) ** 2 + ((j - py) * map_res) ** 2
                    if dist_sq <= buffer_radius ** 2:
                        map_idx = j * map_width + i
                        if map_data[map_idx] > 50:  # Occupied (100) or unknown (-1)
                            return False
                else:  # Outside map bounds
                    return False  # Treat as not clear or unknown
        return True

    def find_random_free_goal(self, current_pose):
        if self.current_map is None:
            self.get_logger().warn("No map received yet to find goals.")
            return None

        map_info = self.current_map.info
        map_data = self.current_map.data
        clearance_check_radius = self.robot_radius + self.clearance_buffer

        for attempt in range(self.goal_attempts_per_cycle):
            # Randomly sample a point within map bounds
            rand_x = random.uniform(map_info.origin.position.x,
                                    map_info.origin.position.x + map_info.width * map_info.resolution)
            rand_y = random.uniform(map_info.origin.position.y,
                                    map_info.origin.position.y + map_info.height * map_info.resolution)

            # Check if this point is clear and has enough buffer
            if self.is_pose_clear(rand_x, rand_y, map_data, map_info, clearance_check_radius):
                # Check distance from current pose
                dist_to_robot = math.sqrt((rand_x - current_pose.pose.position.x) ** 2 + \
                                          (rand_y - current_pose.pose.position.y) ** 2)

                if self.min_goal_distance <= dist_to_robot <= self.max_goal_distance:
                    # Random orientation
                    rand_yaw = random.uniform(-math.pi, math.pi)
                    qx, qy, qz, qw = self.euler_to_quaternion(0, 0, rand_yaw)
                    return self.create_pose_stamped(rand_x, rand_y, 0.0, qx, qy, qz, qw, "map")

        self.get_logger().warn(f"Could not find a valid random goal after {self.goal_attempts_per_cycle} attempts.")
        return None

    def euler_to_quaternion(self, roll, pitch, yaw):
        # From TF2 examples (simplified for 2D yaw)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy
        return x, y, z, w

    def create_pose_stamped(self, x, y, z, qx, qy, qz, qw, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def start_bag_recording(self, suffix=""):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"nav2_debug_{timestamp}{suffix}"
        bag_path = os.path.join(self.bag_directory, bag_name)

        topics_to_record = [
            '/tf', '/tf_static', '/odom', '/imu/data', '/scan', '/map',
            '/amcl_pose', '/global_plan', '/local_plan', '/cmd_vel',
            '/initialpose', '/diagnostics', '/parameter_events',
            '/robot_description', '/joint_states',
            '/map_server/map',  # Redundant if /map is already covered, but good for explicit map saving
            '/costmap_filter_info/control',  # If using costmap filters
            '/costmap_filter_info/filter_mask',
            '/controller_server/global_plan',  # More specific plan topics
            '/planner_server/global_plan',
            '/local_costmap/costmap', '/global_costmap/costmap'  # Actual costmap layers
        ]

        self.get_logger().info(f"Starting ros2 bag recording to: {bag_path}")
        try:
            self.rosbag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_path] + topics_to_record,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            time.sleep(2)  # Give bag recorder a moment to start up
            self.get_logger().info(f"Rosbag process started with PID: {self.rosbag_process.pid}")
        except FileNotFoundError:
            self.get_logger().error("ros2 bag command not found. Make sure ROS 2 is sourced.")
            self.rosbag_process = None
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag recording: {e}")
            self.rosbag_process = None

    def stop_bag_recording(self):
        if self.rosbag_process:
            self.get_logger().info("Stopping ros2 bag recording...")
            self.rosbag_process.terminate()
            try:
                stdout, stderr = self.rosbag_process.communicate(timeout=5)
                if stdout: self.get_logger().info(f"Rosbag stdout: {stdout.decode()}")
                if stderr: self.get_logger().error(f"Rosbag stderr: {stderr.decode()}")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Rosbag process did not terminate gracefully, forcing kill.")
                self.rosbag_process.kill()
            self.rosbag_process = None
            self.get_logger().info("Rosbag recording stopped.")

    def send_new_goal(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Cannot get robot's current pose. Retrying.")
            return

        goal_pose = self.find_random_free_goal(robot_pose)
        if goal_pose is None:
            self.get_logger().warn("Failed to find a suitable goal. Will retry.")
            # If no goal found after many attempts, maybe shut down or wait longer
            return

        self.get_logger().info(
            f"Sending new goal: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}, yaw={math.degrees(self.quaternion_to_euler(goal_pose.pose.orientation)[2]):.2f} deg")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.current_goal_handle = self._action_client.send_goal_async(goal_msg)
        self.current_goal_handle.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            self.current_goal_handle = None
            # Immediately try to send a new goal if current one rejected
            self.send_new_goal()
            return

        self.get_logger().info('Goal accepted.')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result_status = future.result().status
        if result_status == 1:  # GoalStatus.SUCCEEDED
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with status: {result_status}')

        self.current_goal_handle = None  # Clear current goal handle
        self.send_new_goal()  # Immediately send a new goal

    def timer_callback(self):
        # Stop testing if duration exceeded
        if (self.get_clock().now() - self.start_time) > Duration(minutes=self.test_duration_minutes):
            self.get_logger().info(f"Test duration of {self.test_duration_minutes} minutes exceeded. Stopping.")
            self.stop_bag_recording()
            self.destroy_node()
            rclpy.shutdown()
            return

        # If no goal is currently active, try to send one
        if self.current_goal_handle is None:
            self.send_new_goal()

    def quaternion_to_euler(self, q):
        # From geometry_msgs.msg.Quaternion to Euler angles (roll, pitch, yaw)
        # Using scipy.spatial.transform.Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)
        # For a simplified 2D robot, mainly yaw is relevant.
        x, y, z, w = q.x, q.y, q.z, q.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    tester = MapAwareTrajectoryTester()

    # Start bag recording immediately
    tester.start_bag_recording(suffix="_standard_odom_experimental")

    # Give everything a moment to initialize
    tester.get_logger().info("System initializing... Starting goal generation in 10 seconds.")
    time.sleep(10)  # Give map, AMCL, Nav2 time to settle

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Keyboard Interrupt detected, shutting down.")
    finally:
        tester.stop_bag_recording()
        if rclpy.ok():  # Only destroy node if rclpy hasn't already shut down
            tester.destroy_node()
        if rclpy.ok():  # Only shutdown if not already shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()