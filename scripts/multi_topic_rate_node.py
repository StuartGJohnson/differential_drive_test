#!/usr/bin/env python3
# 2025 Stuart Johnson
#
# Gets data rates for multiple simulator nodes (most coding done by GPT4o (OpenAI, June 2025)
#

import rclpy
from rclpy.node import Node
import time
import importlib
import threading
import pandas as pd

def get_stamp(msg):
    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        return msg.header.stamp
    elif hasattr(msg, 'transforms') and msg.transforms:
        return msg.transforms[0].header.stamp
    elif hasattr(msg, 'clock'):
        return msg.clock
    else:
        return None


class RateTracker:
    def __init__(self):
        self.msg_count = 0
        self.wall_start = None
        self.wall_end = None
        self.sim_start = None
        self.sim_end = None

    def update(self, msg):
        now = time.time()
        stamp = get_stamp(msg)
        if stamp is None:
            return

        self.msg_count += 1

        if self.wall_start is None:
            self.wall_start = now
            self.sim_start = stamp
        self.wall_end = now
        self.sim_end = stamp

    def compute_rates(self):
        wall_dur = self.wall_end - self.wall_start
        sim_dur = (
            self.sim_end.sec + self.sim_end.nanosec * 1e-9
            - self.sim_start.sec - self.sim_start.nanosec * 1e-9
        )
        wall_hz = self.msg_count / wall_dur if wall_dur > 0 else 0.0
        sim_hz = self.msg_count / sim_dur if sim_dur > 0 else 0.0
        return wall_hz, sim_hz


def import_ros_msg_type(msg_type_str):
    pkg, _, msg = msg_type_str.partition('/msg/')
    mod = importlib.import_module(f"{pkg}.msg")
    return getattr(mod, msg)


class MultiTopicRateNode(Node):
    def __init__(self, sample_count=100):
        super().__init__('multi_topic_rate_dict_node')
        self.sample_count = sample_count
        self.trackers = {}
        self.subs = []
        self.done_event = threading.Event()
        self.declare_parameter('sim_type', 'gazebo') # Declare the parameter with a default value
        self.sim_type = self.get_parameter('sim_type').get_parameter_value().string_value
        print("sim type is:", self.sim_type)

        # todo: these should be the same!
        if self.sim_type == "isaacsim":
            topic_type_dict = {
                "/imu": "sensor_msgs/msg/Imu",
                "/rgb_camera/rgb": "sensor_msgs/msg/Image",
                "/depth_camera/depth": "sensor_msgs/msg/Image",
                "/scan": "sensor_msgs/msg/LaserScan",
                "/odom": "nav_msgs/msg/Odometry",
                "/clock": "rosgraph_msgs/msg/Clock",
                "/tf": "tf2_msgs/msg/TFMessage"
            }
        elif self.sim_type == "gazebo":
            topic_type_dict = {
                "/imu": "sensor_msgs/msg/Imu",
                "/d435_rgb_camera/image_raw": "sensor_msgs/msg/Image",
                "/d435_depth_camera/image_raw": "sensor_msgs/msg/Image",
                "/scan": "sensor_msgs/msg/LaserScan",
                "/odom": "nav_msgs/msg/Odometry",
                "/clock": "rosgraph_msgs/msg/Clock",
                "/tf": "tf2_msgs/msg/TFMessage",
                "/gt_pose": "geometry_msgs/msg/PoseStamped"
            }
        elif self.sim_type == "robot":
            topic_type_dict = {
                "/imu/data_raw": "sensor_msgs/msg/Imu",
                #"/d435_rgb_camera/image_raw": "sensor_msgs/msg/Image",
                #"/d435_depth_camera/image_raw": "sensor_msgs/msg/Image",
                #"/scan": "sensor_msgs/msg/LaserScan",
                "/odom": "nav_msgs/msg/Odometry",
                #"/clock": "rosgraph_msgs/msg/Clock",
                "/tf": "tf2_msgs/msg/TFMessage"
                #"/gt_pose": "geometry_msgs/msg/PoseStamped"
            }            
        else:
            self.done_event.set()

        for topic, msg_type_str in topic_type_dict.items():
            msg_type = import_ros_msg_type(msg_type_str)
            tracker = RateTracker()
            self.trackers[topic] = tracker

            self.create_subscription(
                msg_type,
                topic,
                lambda msg_in, topic_in=topic: self.callback(msg_in, topic_in),
                10
            )

    def callback(self, msg, topic):
        tracker = self.trackers[topic]
        tracker.update(msg)
        if all(t.msg_count >= self.sample_count for t in self.trackers.values()):
            print("done!")
            self.done_event.set()

    def report(self):
        print(f"\nCollected at least {self.sample_count} messages per topic:\n")
        data_lod = []
        for topic, tracker in self.trackers.items():
            wall_hz, sim_hz = tracker.compute_rates()
            print(f"{topic}: {tracker.msg_count} msgs | {wall_hz:.2f} Hz (wall) | {sim_hz:.2f} Hz (sim)")
            data_lod.append({"topic": topic, "count": tracker.msg_count, "wall Hz": wall_hz, "sim Hz": sim_hz})

        df = pd.DataFrame(data_lod)
        # save the table
        df.to_json("topic_rates.json", orient="records", lines=True)




def main(args=None):
    rclpy.init(args=args)

    sample_count = 200

    node = MultiTopicRateNode(sample_count)

    # Spin until the done_event is set
    while rclpy.ok() and not node.done_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
