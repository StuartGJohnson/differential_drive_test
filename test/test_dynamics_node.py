import time
import unittest

import yaml

from scripts.dynamics_node import TimeTrajectory
from scripts.dynamics_node import Data, CircleFit
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Quaternion, Vector3
import numpy as np
import matplotlib.pyplot as plt
import os

class MyTestCase(unittest.TestCase):
    def test_data(self):
        data = Data("thingy")
        pose = PoseStamped()
        p = Point(x=0.0,y=0.0,z=0.0)
        q = Quaternion(x=0.0,y=0.0,z=1.0,w=0.0)
        v = Vector3(x=0.0,y=0.0,z=0.0)
        pose.pose.position = p
        pose.pose.orientation = q
        pose_t = TransformStamped()
        pose_t.transform.translation = v
        pose_t.transform.rotation = q
        np = 100
        dt = 0.1
        for i in range(0, np):
            t = dt * i
            data.add_data_from_pose(t, pose)
            data.add_data_from_transform(t, pose_t)
        #print(data.to_dict())
        new_data = data.crop(1.0)
        lx = len(data.x)
        lxn = len(new_data.x)
        print("data lengths, pre/post crop: ", lx, lxn)
        diff = lx - lxn
        diff_pred = lx -(np*2 - np*dt*4)
        print("data length diffs, obs, pred: ", diff, diff_pred)
        self.assertTrue(abs(diff - diff_pred)<5)  # add assertion here

    def test_trajectory(self, do_plots = False):
        tt = TimeTrajectory()
        tvec = []
        vvec = []
        for t in np.linspace(0.0, tt.max_time, 100):
            tvec.append(t)
            vvec.append(tt.compute(t))
        tvec = np.array(tvec)
        vvec = np.array(vvec)
        # this should be a smooth curve with rampups/downs, etc
        if do_plots:
            plt.figure()
            plt.plot(tvec, vvec)
            plt.grid()
            plt.show()

    def test_circle_fit(self, do_plots=False):
        # check partial arc
        np.random.seed(42)
        theta = np.linspace(0, np.pi/3, 100)
        x = np.cos(theta) * 42.0 + 3.7 + 0.1*np.random.randn(100)
        y = np.sin(theta) * 42.0 + 12.2 + 0.1*np.random.randn(100)
        cf = CircleFit(x,y)
        r = cf.solve()
        if do_plots:
            plt.figure()
            plt.plot(x, y, 'r+')
            plt.axis('equal')
            plt.show()
        self.assertLess(np.abs(r-42), 1.0)
        print(r)

    def test_circle_fit2(self, do_plots=False):
        np.random.seed(43)
        # check straight line
        x = np.ones((100,)) + 0.01*np.random.randn(100)
        y = np.linspace(-0.5,0.5,100) + 0.01*np.random.randn(100)
        cf = CircleFit(x,y)
        r = cf.solve()
        if do_plots:
            plt.figure()
            plt.plot(x, y, 'r+')
            plt.axis('equal')
            plt.show()
        self.assertGreater(r, 10.0)
        print(r)

    def test_list(self):
        t = np.ones((100,))
        t = list(t)
        t_crop = t[20:80]
        print(len(t_crop))

    @unittest.skip("for checking legend generation in plots")
    def test_plot(self):
        # for checking legend generation
        np.random.seed(44)
        theta = np.linspace(0, np.pi/3, 100)
        x = np.cos(theta) * 42.0 + 3.7 + 0.1*np.random.randn(100)
        y = np.sin(theta) * 42.0 + 12.2 + 0.1*np.random.randn(100)
        plt.figure()
        plt.plot(x, y, 'r+')
        plt.plot(x+1,y+2,'g+')
        plt.axis('equal')
        plt.legend(['data','ground truth'])
        plt.show()

    @unittest.skip("think about this before running it, it generates files")
    def test_filestamp(self):
        run_time = time.strftime("%Y%m%d_%H%M%S")
        time_dir = os.path.join(os.getenv("HOME"), "sc" + run_time)
        os.makedirs(time_dir, exist_ok=True)
        time_name = os.path.join(time_dir, "sc" + run_time + '.txt')
        with open(time_name, 'w') as f:
            print(time_name)
            print(time_name,file=f)

    @unittest.skip("think about this before running it, it generates files")
    def test_file_creation(self):
        package_dir = '/home/sjohnson/ros2_ws/src/differential_drive_test'
        # dry run of the files created
        run_time = time.strftime("%Y%m%d_%H%M%S")
        time_dir = "sc" + run_time
        cfg_file = os.path.join(package_dir,"config/sim_check.yml")
        cfg_file_leaf = os.path.basename(cfg_file)
        with open(cfg_file, 'r') as file:
            cfg = yaml.safe_load(file)
        home_dir = cfg["output_home_dir"]
        test_home_dir = os.path.join(home_dir,time_dir)
        os.makedirs(home_dir, exist_ok=True)
        os.makedirs(test_home_dir, exist_ok=False)
        cfg_file_out = os.path.join(test_home_dir,cfg_file_leaf)
        with open(cfg_file_out, 'w') as file:
            yaml.dump(cfg, file, sort_keys=False)

        # cycle through the tests
        for test in cfg['tests']:
            test_dir = os.path.join(test_home_dir,test['name'])
            os.makedirs(test_dir, exist_ok=True)

    def test_strings(self):
        sim_type="gazebo"
        lstr = ["this","is",f"test={sim_type}"]
        print(lstr)

    def test_wtf(self):
        thing = "this is:" + ".a thing"
        print(thing)

if __name__ == '__main__':
    unittest.main(verbosity=2)
