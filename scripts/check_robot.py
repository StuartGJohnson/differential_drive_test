#!/usr/bin/env python3
# 2025 Stuart Johnson
#
# Check robot dynamics by commanding a series of constant radius turns
# and observing behavior via various sensors.
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
import shutil
import glob
from nav_msgs.msg import Odometry
from robot_tools import reset_robot, find_robot

def wait_for_robot(timeout=20.0):
    rclpy.init()
    node = rclpy.create_node('robot_waiter')
    got_robot = False

    def cb(msg):
        nonlocal got_robot
        got_robot = True

    sub = node.create_subscription(Odometry, '/odom', cb, 10)
    start_time = time.time()
    while not got_robot and (time.time() - start_time < timeout):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return got_robot

def startup_robot(robot_file: str, world_file: str) -> subprocess.Popen:
    # if first_pass:
    print("Starting robot...")
    rob_proc = subprocess.Popen(
        ["ros2",
        "launch",
        "ugv_bringup",
        "bringup_base.launch.py"],
        #f"world:={world_file}",
        #f"robot:={robot_file}"],
        preexec_fn=os.setsid)
    return rob_proc

def main():
    parser = argparse.ArgumentParser(description='Simple orchestrator')
    parser.add_argument('--config', type=str, required=True, help='Path to config file')
    args = parser.parse_args()
    # first, see if we have a robot
    robot_tty = find_robot()
    if robot_tty is None:
        print("Can't find a robot.")
        return
    print("Robot found ... resetting ...")
    reset_robot(robot_tty)
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

    # single robot start-up, for now
    # should we have some sort of a reset?
    robot_proc = startup_robot("", "")
    sim_type = 'robot'

    # cycle through the tests
    first_pass = True
    for test in cfg['tests']:
        test_dir = os.path.join(test_home_dir, test['name'])
        os.makedirs(test_dir, exist_ok=True)
        world_file = test['world_file']
        robot_file = test['robot_file']

        print("Waiting for robot...")
        if not wait_for_robot():
            print("Timed out waiting for robot.")
            os.killpg(os.getpgid(robot_proc.pid), signal.SIGINT)
            return

        if first_pass:
            # let tf settle a bit for th transform display code
            time.sleep(5)
            first_pass = False
            print("Running transform display code...")
            subprocess.run(["ros2",
                            "run",
                            "tf2_tools",
                            "view_frames"], check=True)
            latest_gv = max(glob.glob("frames_*.gv"), key=os.path.getmtime)
            gv_target_file = os.path.join(test_home_dir,f"frames_{sim_type}.gv")
            gv_target_file_clean = os.path.join(test_home_dir,f"frames_{sim_type}_clean.gv")
            svg_target_file = os.path.join(test_home_dir,f"frames_{sim_type}.svg")
            jpg_target_file = os.path.join(test_home_dir,f"frames_{sim_type}.jpg")
            shutil.copy(latest_gv, gv_target_file)
            # Clean the .gv file and convert to SVG/JPG
            with open(gv_target_file) as f:
                lines = f.readlines()
            with open(gv_target_file_clean, "w") as f:
                for line in lines:
                    # Remove edge labels
                    if "label=" in line:
                        line = line.split("[")[0] + ";\n"
                    f.write(line)
            # convert to svg and jpg
            subprocess.run(["dot",
                            "-Tsvg",
                            gv_target_file_clean,
                            "-o",
                            svg_target_file], check=True)
            subprocess.run(["dot",
                            "-Tjpg",
                            gv_target_file_clean,
                            "-o",
                            jpg_target_file], check=True)

            # now check publish rates
            print("Running topic rate node...")
            subprocess.run(["ros2",
                            "run",
                            "differential_drive_test",
                            "multi_topic_rate_node.py",
                            "--ros-args",
                            "-p", f"sim_type:={sim_type}",
                            ])
            shutil.copy("topic_rates.json", os.path.join(test_home_dir,"topic_rates.json"))

        print("Running sensor check node...")
        subprocess.run(["ros2",
                        "run",
                        "differential_drive_test",
                        "sensors_node.py",
                        "--ros-args",
                        "-p", f"sim_type:={sim_type}",
                        "-p", f"report_dir:={test_dir}",
                        ])

        if test['max_time'] > 0.0:
            print("Running data collector...")
            subprocess.run(["ros2",
                            "run",
                            "differential_drive_test",
                            "dynamics_node.py",
                            "--ros-args",
                            "-p", "use_sim_time:=false",
                            "-p", f"sim_type:={sim_type}",
                            "-p", f"report_dir:={test_dir}",
                            "-p", f"max_time:={test['max_time']}",
                            "-p", f"rise_time:={test['rise_time']}",
                            "-p", f"angular_velocity:={test['angular_velocity']}",
                            "-p", f"linear_velocity:={test['linear_velocity']}",
                            ])
            print("Resetting robot...")
            reset_robot(robot_tty)

    print("Shutting down robot...")
    os.killpg(os.getpgid(robot_proc.pid), signal.SIGINT)
    robot_proc.wait()

if __name__ == '__main__':
    main()