#!/usr/bin/env python3
# 2025 Stuart Johnson
#
# Check robot dynamics by commanding a series of constant radius turns
# and observing ground truth behavior in the simulator.
#
import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from ament_index_python.packages import get_package_share_directory
import yaml
import argparse

def wait_for_clock(timeout=20.0):
    rclpy.init()
    node = rclpy.create_node('clock_waiter')
    got_clock = False

    def cb(msg):
        nonlocal got_clock
        got_clock = True

    sub = node.create_subscription(Clock, '/clock', cb, 10)
    start_time = time.time()
    while not got_clock and (time.time() - start_time < timeout):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return got_clock

def main():
    parser = argparse.ArgumentParser(description='Simple orchestrator')
    parser.add_argument('--config', type=str, required=True, help='Path to config file')
    args = parser.parse_args()
    package_dir = get_package_share_directory("differential_drive_test")
    run_time = time.strftime("%Y%m%d_%H%M%S")
    time_dir = "sc" + run_time
    if args.config[0] == '/':
        # allow absolute path
        cfg_file = args.config
    else:
        # otherwise, in package config
        cfg_file = os.path.join(package_dir, args.config)
    cfg_file_leaf = os.path.basename(cfg_file)
    with open(cfg_file, 'r') as file:
        cfg = yaml.safe_load(file)
    home_dir = cfg["output_home_dir"]
    test_home_dir = os.path.join(home_dir, time_dir)
    os.makedirs(home_dir, exist_ok=True)
    os.makedirs(test_home_dir, exist_ok=False)
    cfg_file_out = os.path.join(test_home_dir, cfg_file_leaf)
    with open(cfg_file_out, 'w') as file:
        yaml.dump(cfg, file, sort_keys=False)

    # cycle through the tests
    for test in cfg['tests']:
        test_dir = os.path.join(test_home_dir, test['name'])
        os.makedirs(test_dir, exist_ok=True)
        sim_type = test['sim_type']
        world_file = test['world_file']

        print("Starting simulator...")
        if sim_type == "gazebo":
            sim_proc = subprocess.Popen(
                ["ros2",
                 "launch",
                 "gazebo_differential_drive_robot_4wheel",
                 "robot.launch.py",
                 f"world:={world_file}"],
                preexec_fn=os.setsid)
        elif sim_type == "isaacsim":
            sim_proc = subprocess.Popen(
                ["ros2",
                 "launch",
                 "isaacsim_differential_drive_robot_4wheel",
                 "isaac_sim.launch.py",
                 f"world:={world_file}"],
                preexec_fn=os.setsid)
        else:
            print("unknown simulator type")
            break

        print("Waiting for /clock...")
        if not wait_for_clock():
            print("Timed out waiting for clock.")
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
            return

        print("Running sensor check node...")
        subprocess.run(["ros2",
                        "run",
                        "differential_drive_test",
                        "sensors_node.py",
                        "--ros-args",
                        "-p", f"sim_type:={sim_type}",
                        "-p", f"report_dir:={test_dir}",
                        ])

        print("Running data collector...")
        subprocess.run(["ros2",
                        "run",
                        "differential_drive_test",
                        "dynamics_node.py",
                        "--ros-args",
                        "-p", f"sim_type:={sim_type}",
                        "-p", f"report_dir:={test_dir}",
                        "-p", f"max_time:={test['max_time']}",
                        "-p", f"rise_time:={test['rise_time']}",
                        "-p", f"angular_velocity:={test['angular_velocity']}",
                        "-p", f"linear_velocity:={test['linear_velocity']}",
                        ])

        print("Shutting down simulator...")
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        sim_proc.wait()

if __name__ == '__main__':
    main()