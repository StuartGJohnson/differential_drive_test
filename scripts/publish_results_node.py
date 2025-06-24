#!/usr/bin/env python3
# 2025 Stuart Johnson
#
# collects and publishes sim data to README.md tables.
# also collects data and copies to the repo
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
import shutil
import glob
import pandas as pd
import pathlib


def update_markdown(md_path: str, df: pd.DataFrame, md_tag: str) -> None:
    #df = pd.read_json("report.json", orient="records", lines=True)
    # this will fail in columns which have varying type (e.g., with a "-" and nums)
    #dfr = df.round(2)
    markdown_table = df.to_markdown(index=False)

    # 2. Load the README and inject the table between markers
    with open(md_path, "r") as f:
        lines = f.readlines()

    try:
        start_idx = lines.index(f"<!-- {md_tag}_START -->\n") + 1
        end_idx = lines.index(f"<!-- {md_tag}_END -->\n")
    except ValueError:
        raise RuntimeError("README.md is missing <!-- TABLE_START --> or <!-- TABLE_END --> markers.")

    # 3. Replace old table content
    table_block = markdown_table.splitlines(keepends=False)
    table_block = [line + "\n" for line in table_block]

    new_lines = lines[:start_idx] + table_block + lines[end_idx:]

    # 4. Save updated README
    with open(md_path, "w") as f:
        f.writelines(new_lines)

def main():
    parser = argparse.ArgumentParser(description='data collect and publish')
    parser.add_argument('--config', type=str, required=True, help='Path to config file')
    args = parser.parse_args()
    package_dir = get_package_share_directory("differential_drive_test")
    #package_dir = "../"
    if args.config[0] == '/':
        # allow absolute path
        cfg_file = args.config
    else:
        # otherwise, in package config
        cfg_file = os.path.join(package_dir, args.config)
    #cfg_file = "../config/publish.yml"
    with open(cfg_file, 'r') as file:
        cfg = yaml.safe_load(file)
    package_update_dir = cfg["package_update_dir"]
    update_target = os.path.join(package_update_dir, "README.md")
    pkg_dir = os.path.join(package_update_dir, "sim_check")
    os.makedirs(pkg_dir, exist_ok=True)
    isaac_pkg_dir = os.path.join(pkg_dir, "isaac")
    gazebo_pkg_dir = os.path.join(pkg_dir, "gazebo")
    gazebo_home_dir = cfg["output_home_dir_gazebo"]
    gazebo_time_dir = cfg["time_dir_gazebo"]
    isaac_home_dir = cfg["output_home_dir_isaac"]
    isaac_time_dir = cfg["time_dir_isaac"]
    gazebo_dir = os.path.join(gazebo_home_dir, gazebo_time_dir)
    isaac_dir = os.path.join(isaac_home_dir, isaac_time_dir)
    # copy data
    shutil.copytree(gazebo_dir, gazebo_pkg_dir, dirs_exist_ok=True)
    shutil.copytree(isaac_dir, isaac_pkg_dir, dirs_exist_ok=True)

    # test_1
    df1 = pd.read_json(os.path.join(gazebo_pkg_dir, "test_1/dynamics_report.json"), orient="records", lines=True).round(2)
    df2 = pd.read_json(os.path.join(isaac_pkg_dir, "test_1/dynamics_report.json"), orient="records", lines=True).round(2)
    df_all = pd.concat([df1, df2], axis=0, ignore_index=True)
    update_markdown(update_target, df_all, "TEST1_TABLE")

    # test_2
    df1 = pd.read_json(os.path.join(gazebo_pkg_dir, "test_2/dynamics_report.json"), orient="records", lines=True).round(2)
    df2 = pd.read_json(os.path.join(isaac_pkg_dir, "test_2/dynamics_report.json"), orient="records", lines=True).round(2)
    df_all = pd.concat([df1, df2], axis=0, ignore_index=True)
    update_markdown(update_target, df_all, "TEST2_TABLE")

    # test_3
    df1 = pd.read_json(os.path.join(gazebo_pkg_dir, "test_3/dynamics_report.json"), orient="records", lines=True).round(2)
    df2 = pd.read_json(os.path.join(isaac_pkg_dir, "test_3/dynamics_report.json"), orient="records", lines=True).round(2)
    df_all = pd.concat([df1, df2], axis=0, ignore_index=True)
    update_markdown(update_target, df_all, "TEST3_TABLE")

    # test_4
    df1 = pd.read_json(os.path.join(gazebo_pkg_dir, "test_4/dynamics_report.json"), orient="records", lines=True).round(2)
    df2 = pd.read_json(os.path.join(isaac_pkg_dir, "test_4/dynamics_report.json"), orient="records", lines=True).round(2)
    df_all = pd.concat([df1, df2], axis=0, ignore_index=True)
    update_markdown(update_target, df_all, "TEST4_TABLE")

    # gazebo data rate
    df = pd.read_json(os.path.join(gazebo_pkg_dir, "topic_rates.json"), orient="records", lines=True).round(2)
    update_markdown(update_target, df, "GAZEBO_DATA_RATE_TABLE")
    # isac data rate
    df = pd.read_json(os.path.join(isaac_pkg_dir, "topic_rates.json"), orient="records", lines=True).round(2)
    update_markdown(update_target, df, "ISAAC_DATA_RATE_TABLE")

if __name__ == '__main__':
    main()