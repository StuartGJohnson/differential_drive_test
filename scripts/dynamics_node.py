#!/usr/bin/env python3
# 2025 Stuart Johnson
#
# Dynamics test for simulation and real robots as a ros2 node. THis node commands trajectories
# and observes robot pose. The purpose of these tests is to quantify drive dynamics in real and
# actual cases.
#
from typing import Tuple, Iterator, List, Type
import time
import sys
import rclpy
import scipy.io
from scipy import optimize
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.service import Service
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.spatial.transform.rotation import Rotation as rot
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
import pandas as pd

class CircleFit:
    # see https://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
    def __init__(self, x, y):
        self.x = np.array(x)
        self.y = np.array(y)

    def calc_r(self, c: np.ndarray) -> np.ndarray:
        return np.sqrt((self.x - c[0])**2 + (self.y - c[1])**2)

    def f_2(self, c: np.ndarray):
        ri = self.calc_r(c)
        return ri - np.mean(ri)

    def solve(self):
        x_m = np.mean(self.x)
        y_m = np.mean(self.y)
        center_estimate = np.array([x_m, y_m])
        center, ier = optimize.leastsq(self.f_2, center_estimate)
        radius = np.mean(self.calc_r(center))
        return radius

class Data:
    def __init__(self, tag: str):
        self.angle = []
        self.x = []
        self.y = []
        self.z = []
        self.t = []
        self.tag = tag

    # sigh, if only python had method overloading!
    def add_data_from_transform(self, timestamp: float, pose: TransformStamped):
        if pose is not None:
            ptr = pose.transform.rotation
            # angle about z axis - note this will still need
            # to be unwrapped
            r = rot.from_quat([ptr.x,ptr.y,ptr.z,ptr.w])
            self.angle.append(r.as_rotvec()[2])
            self.x.append(pose.transform.translation.x)
            self.y.append(pose.transform.translation.y)
            self.z.append(pose.transform.translation.z)
            self.t.append(timestamp)

    def add_data_from_pose(self, timestamp: float, pose: PoseStamped):
        if pose.pose is not None:
            ptr = pose.pose.orientation
            # angle about z axis - note this will still need
            # to be unwrapped
            r = rot.from_quat([ptr.x,ptr.y,ptr.z,ptr.w])
            self.angle.append(r.as_rotvec()[2])
            self.x.append(pose.pose.position.x)
            self.y.append(pose.pose.position.y)
            self.z.append(pose.pose.position.z)
            self.t.append(timestamp)

    def add_data_from_odom(self, timestamp: float, pose: Odometry):
        if pose.pose is not None:
            ptr = pose.pose.pose.orientation
            # angle about z axis - note this will still need
            # to be unwrapped
            r = rot.from_quat([ptr.x,ptr.y,ptr.z,ptr.w])
            self.angle.append(r.as_rotvec()[2])
            self.x.append(pose.pose.pose.position.x)
            self.y.append(pose.pose.pose.position.y)
            self.z.append(pose.pose.pose.position.z)
            self.t.append(timestamp)

    def to_dict(self) -> dict:
        data_pose_angle = np.array(self.angle)
        data_time = np.array(self.t)
        data_x = np.array(self.x)
        data_y = np.array(self.y)
        data_z = np.array(self.z)
        tag2 = self.tag
        if self.tag != "":
            tag2 = self.tag + "_"
        mat_data = {
            tag2 + "pose_angle": data_pose_angle,
            tag2 + "pose_time": data_time,
            tag2 + "pose_x": data_x,
            tag2 + "pose_y": data_y,
            tag2 + "pose_z": data_z,
        }
        return mat_data

    def crop(self, t_crop) -> "Data":
        # trim off the first and last t_crop items
        t_tmp = np.array(self.t)
        t_max = np.max(t_tmp)
        ind_start = np.argmin(t_tmp < t_crop)
        ind_end = np.argmin(t_tmp < (t_max - t_crop))
        new_data = Data(self.tag)
        new_data.t = self.t[ind_start:ind_end]
        new_data.x = self.x[ind_start:ind_end]
        new_data.y = self.y[ind_start:ind_end]
        new_data.z = self.z[ind_start:ind_end]
        new_data.angle = self.angle[ind_start:ind_end]
        return new_data

class TimeTrajectory:
    def __init__(self, rise_time=2.5, max_time=10.0):
        self.rise_time = rise_time
        self.max_time = max_time
        # precompute logit parameters
        # logit is at alpha value at beginning of rise_time
        alpha = .001
        self.logit_k = 2.0 * np.log((1-alpha)/alpha) / self.rise_time
        self.start_logit_t0 = self.rise_time/2.0
        self.end_logit_t0 = self.max_time - self.rise_time/2.0

    def compute(self, t: float) -> float:
        # compute current value of trajectory using start and end logits
        logit_start = 1.0/(1.0 + np.exp(-self.logit_k*(t - self.start_logit_t0)))
        logit_end = 1.0/(1.0 + np.exp(self.logit_k*(t - self.end_logit_t0)))
        return logit_start * logit_end

class DynamicsNode(Node):
    def __init__(self):
        super().__init__(
            'dynamics_node'
        )
        self.start_time = np.nan
        self.wall_start_time = np.nan
        self.wall_stop_time = np.nan
        self.gt_start_time = np.nan
        self.gt_current_time = np.nan

        default_report_dir = os.path.join(os.getenv("HOME"),'sim_check')
        self.declare_parameter('report_dir', default_report_dir) # Declare the parameter with a default value
        self.report_dir = self.get_parameter('report_dir').get_parameter_value().string_value
        self.declare_parameter('max_time', 10.0) # Declare the parameter with a default value
        max_time = self.get_parameter('max_time').get_parameter_value().double_value
        self.declare_parameter('rise_time', 2.5) # Declare the parameter with a default value
        rise_time = self.get_parameter('rise_time').get_parameter_value().double_value
        self.trajectory = TimeTrajectory(rise_time, max_time)

        self.declare_parameter('linear_velocity', 1.0) # Declare the parameter with a default value
        self.linear_vel = self.get_parameter('linear_velocity').get_parameter_value().double_value
        self.declare_parameter('angular_velocity', 1.0) # Declare the parameter with a default value
        self.angular_vel = self.get_parameter('angular_velocity').get_parameter_value().double_value
        # trajectory parameters
        # m/s and radians/s
        # TimeTrajectory smoothes this constant vel trajectory
        #self.linear_vel = 1.0
        #self.angular_vel = 2.0

        self.data = Data("")
        self.gt_data = Data("gt")
        self.done = False
        self.declare_parameter('sim_type', 'gazebo') # Declare the parameter with a default value
        self.sim_type = self.get_parameter('sim_type').get_parameter_value().string_value
        print("sim type is:", self.sim_type)
        if self.sim_type == 'isaacsim':
            self.adj_omega = 1.0
            self.source_xform_frame = 'odom'
        elif self.sim_type == 'gazebo':
            self.adj_omega = 1.0
            self.source_xform_frame = 'differential_drive_robot_4wheel/odom'
        elif self.sim_type == 'robot':
            self.adj_omega = 1.0
            self.source_xform_frame = 'odom'
        else:
            print("unknown simulator type")
            self.done=True
            return
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        print("use_sim_time is: ", use_sim_time)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.listener_tf = self.create_subscription(TFMessage, '/tf', self.process_tf, 10)
        # todo: for isaacsim, is seems both the tf value and the odom value are ground truth. fix
        if self.sim_type == "isaacsim":
            # this is the same as the /tf value for isaacsim
            self.listener_gt = self.create_subscription(Odometry, '/odom', self.process_odom, 10)
        elif self.sim_type == 'robot':
            self.listener_gt = self.create_subscription(Odometry, '/odom', self.process_odom, 10)
        else:
            self.listener_gt = self.create_subscription(PoseStamped, '/gt_pose', self.process_gt, 10)
        self.cmd_timer = self.create_timer(0.1, self.publish_command)

    def extract_transform(self, tf_message: TFMessage, source_frame: str, target_frame:str) -> Tuple[float, TransformStamped | None]:
        """
        Extracts a specific transform from a TFMessage.

        Args:
            tf_message: The TFMessage object.
            source_frame: The ID of the source frame.
            target_frame: The ID of the target frame.

        Returns:
            A TransformStamped object if the transform is found, otherwise None.
        """
        for transform in tf_message.transforms:
            timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1.e9
            if transform.header.frame_id == source_frame and transform.child_frame_id == target_frame:
                return timestamp, transform
        return timestamp, None

    def publish_command(self):
        current_time = self.get_clock().now().nanoseconds / 1.e9
        current_wall_time = time.time()
        if current_time == 0:
            return
        if np.isnan(self.start_time):
            self.start_time = current_time
            self.wall_start_time = current_wall_time
        time_elapsed = current_time - self.start_time
        self.wall_stop_time = current_wall_time
        # compute the logistic trajectory
        current_scale = self.trajectory.compute(time_elapsed)
        msg = Twist()
        if time_elapsed <= self.trajectory.max_time:
            msg.linear.x = self.linear_vel * current_scale
            msg.angular.z = self.angular_vel * current_scale
            self.publisher.publish(msg)
        else:
            self.destroy_timer(self.cmd_timer)
            self.done = True

    def process_tf(self, tf_msg: TFMessage):
        if np.isnan(self.start_time):
            return
        stamp, pose = self.extract_transform(
            tf_msg,
            self.source_xform_frame,
            'base_footprint')
        time_elapsed = stamp - self.start_time
        self.data.add_data_from_transform(time_elapsed, pose)

    def process_gt(self, pose: PoseStamped):
        if np.isnan(self.start_time):
            return
        stamp = pose.header.stamp.sec + pose.header.stamp.nanosec/1.e9
        time_elapsed = stamp - self.start_time
        self.gt_data.add_data_from_pose(time_elapsed, pose)

    def process_odom(self, pose: Odometry):
        if np.isnan(self.start_time):
            return
        stamp = pose.header.stamp.sec + pose.header.stamp.nanosec/1.e9
        time_elapsed = stamp - self.start_time
        self.gt_data.add_data_from_odom(time_elapsed, pose)

    def serialize(self):
        # files
        mat_file = os.path.join(self.report_dir, "dynamics.mat")
        report_file = os.path.join(self.report_dir, "dynamics_report.txt")
        report_file_json = os.path.join(self.report_dir, "dynamics_report.json")
        plot_root = os.path.join(self.report_dir, "dynamics")

        # write the time series to a matlab .mat file - for matlab analysis
        if len(self.data.t) == 0:
            return
        data_dict = self.data.to_dict()
        data_dict.update(self.gt_data.to_dict())
        scipy.io.savemat(mat_file, data_dict)

        # analyze the data for a  general analysis of trajectory curvature
        # and integrated turning rate - note this assumes a lot about what
        # the simulated trajectory looks like

        # crop the data to steady state intervals
        cropped_data = self.data.crop(self.trajectory.rise_time)
        cropped_gt_data = self.gt_data.crop(self.trajectory.rise_time)

        # compute heading metrics
        data_angle = np.unwrap(cropped_data.angle)
        gt_angle = np.unwrap(cropped_gt_data.angle)
        data_angle_change = data_angle[-1] - data_angle[0]
        gt_angle_change = gt_angle[-1] - gt_angle[0]

        # fit radii
        data_fit = CircleFit(cropped_data.x, cropped_data.y)
        gt_fit = CircleFit(cropped_gt_data.x, cropped_gt_data.y)
        data_r = data_fit.solve()
        gt_r = gt_fit.solve()

        # suggest calibration
        if data_r > 0.1:
            est1 = float(gt_r)/float(data_r)
            est2 = data_angle_change/gt_angle_change
            mean_est = (est1+est2)/2.0
        else:
            mean_est =  data_angle_change / gt_angle_change

        delta_time = np.max(self.data.t)-np.min(self.data.t)
        delta_gt_time = np.max(self.gt_data.t) - np.min(self.gt_data.t)
        delta_wall_time = self.wall_stop_time - self.wall_start_time
        # to stdout
        self.make_report(data_r, gt_r,
                         data_angle_change, gt_angle_change,
                         mean_est,
                         delta_time, delta_gt_time,
                         delta_wall_time, "", "")

        # serialize to file(s)
        self.make_report(data_r, gt_r,
                data_angle_change, gt_angle_change,
                mean_est,
                delta_time, delta_gt_time,
                delta_wall_time, report_file, report_file_json)

        self.plot(plot_root)

    def make_report(self,
                    data_r, gt_r,
                    data_angle_change, gt_angle_change,
                    mean_est,
                    delta_time, delta_gt_time,
                    delta_wall_time, report_file="", pandas_path=""):
        f = None
        if report_file != "":
            f = open(report_file, 'w')
        print("data, ground truth turn radii (meters): ", data_r, gt_r, file=f)
        print("data, ground truth angle change (radians): ", data_angle_change, gt_angle_change, file=f)
        print("wheel separation calibration factor: ", mean_est, file=f)
        print('data, ground truth sim time change(seconds): ', delta_time, delta_gt_time, file=f)
        print('wall time change(seconds): ', delta_wall_time, file=f)
        if f is not None:
            f.close()
        # and to pandas
        if pandas_path != "":
            data_lod = [{
                "sim_type": self.sim_type,
                "odom turn radius(m)": data_r if self.sim_type == 'gazebo' else "-",
                "gt turn radius(m)": gt_r,
                "odom heading change(rad)": data_angle_change if self.sim_type == 'gazebo' else "-",
                "gt heading change(rad):": gt_angle_change,
                "sim time change(s)": delta_time,
                "wall time change(s)": delta_wall_time}]
            df = pd.DataFrame(data_lod)
            # save the table
            df.to_json(pandas_path, orient="records", lines=True)

    def plot(self, filename_base: str):
        # plot up the data and save to files for inclusion in
        # documents, etc
        x = np.array(self.data.x)
        y = np.array(self.data.y)
        t = np.array(self.data.t)
        gtx = np.array(self.gt_data.x)
        gty = np.array(self.gt_data.y)
        gtt = np.array(self.gt_data.t)
        xref = x[0]
        yref = y[0]
        tref = t[0]
        gt_xref = gtx[0]
        gt_yref = gty[0]
        gt_tref = gtt[0]
        plt.figure()
        if self.sim_type == 'gazebo':
            plt.plot(x - xref, y - yref, 'r+')
            plt.plot(gtx - gt_xref, gty - gt_yref, 'g+')
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('robot trajectory')
            plt.axis('equal')
            plt.legend(['int. odom','ground truth'])
        elif self.sim_type == 'isaacsim':
            # isaacsim is only giving me ground truth
            #plt.plot(x - xref, y - yref, 'r+')
            plt.plot(gtx - gt_xref, gty - gt_yref, 'g+')
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('robot trajectory')
            plt.axis('equal')
            plt.legend(['ground truth'])
        elif self.sim_type == 'robot':
            # isaacsim is only giving me odom (for now)
            #plt.plot(x - xref, y - yref, 'r+')
            plt.plot(gtx - gt_xref, gty - gt_yref, 'g+')
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('robot trajectory')
            plt.axis('equal')
            plt.legend(['int. odom'])
        plt.savefig(filename_base + "_traj.jpg")
        #plt.show()

        plt.figure()
        angle = np.unwrap(self.data.angle)
        gt_angle = np.unwrap(self.gt_data.angle)
        if self.sim_type == 'gazebo':
            plt.plot(t-tref, angle, 'r+')
            plt.plot(gtt-gt_tref, gt_angle, 'g+')
            plt.xlabel('sim time (sec)')
            plt.ylabel('heading angle (radians)')
            plt.title('robot heading angle')
            plt.legend(['int. odom', 'ground truth'])
        elif self.sim_type == 'isaacsim':
            #plt.plot(t-tref, angle, 'r+')
            plt.plot(gtt-gt_tref, gt_angle, 'g+')
            plt.xlabel('sim time (sec)')
            plt.ylabel('heading angle (radians)')
            plt.title('robot heading angle')
            plt.legend(['ground truth'])
        if self.sim_type == 'robot':
            #plt.plot(t-tref, angle, 'r+')
            plt.plot(gtt-gt_tref, gt_angle, 'g+')
            plt.xlabel('sim time (sec)')
            plt.ylabel('heading angle (radians)')
            plt.title('robot heading angle')   
            plt.legend(['int. odom'])   
        plt.savefig(filename_base + "_angle.jpg")
        #plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = DynamicsNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.serialize()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





