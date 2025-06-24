# differential_drive_test

A ROS2 package/node for orchestrating tests of ROS2 simulation nodes and collecting results.

## Installation

## Usage

## Simulator Comparisons

### Robot self portrait

#### RGB Camera
<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/rgb_image.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/rgb_image.jpg" width="90%"/></td>
  </tr>
</table>

#### Depth Camera
<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/depth_image.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/depth_image.jpg" width="90%"/></td>
  </tr>
</table>

### Transform trees

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/frames_gazebo.jpg" width="90%"/></td>
  </tr>
  <tr>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/isaac/frames_isaacsim.jpg" width="90%"/></td>
  </tr>
</table>

### Sensors - Depth Camera

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_1/depth_image.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_1/depth_image.jpg" width="90%"/></td>
  </tr>
</table>

### Sensors - RGB Camera

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_1/rgb_image.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_1/rgb_image.jpg" width="90%"/></td>
  </tr>
</table>

### Sensors - Lidar

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_1/lidar.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_1/lidar.jpg" width="90%"/></td>
  </tr>
</table>

### Dynamics - Open-Loop control 0m (in-place pivot) radius turn
<!-- TEST4_TABLE_START -->
| sim_type   | odom turn radius(m)   |   gt turn radius(m) | odom heading change(rad)   |   gt heading change(rad): |   sim time change(s) |   wall time change(s) |
|:-----------|:----------------------|--------------------:|:---------------------------|--------------------------:|---------------------:|----------------------:|
| gazebo     | 0                     |                0.04 | 5.03                       |                      5.1  |                10.04 |                 14.78 |
| isaacsim   | -                     |                0.05 | -                          |                      0.94 |                10.05 |                 24.9  |
<!-- TEST4_TABLE_END -->

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_4/dynamics_angle.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_4/dynamics_angle.jpg" width="90%"/></td>
  </tr>
</table>

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_4/dynamics_traj.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_4/dynamics_traj.jpg" width="90%"/></td>
  </tr>
</table>

### Dynamics - Open-Loop control 1m radius turn

<!-- TEST1_TABLE_START -->
| sim_type   | odom turn radius(m)   |   gt turn radius(m) | odom heading change(rad)   |   gt heading change(rad): |   sim time change(s) |   wall time change(s) |
|:-----------|:----------------------|--------------------:|:---------------------------|--------------------------:|---------------------:|----------------------:|
| gazebo     | 1.0                   |                1    | 5.01                       |                      4.99 |                10.02 |                 14.6  |
| isaacsim   | -                     |               46.01 | -                          |                      0.12 |                10.07 |                 24.51 |
<!-- TEST1_TABLE_END -->

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_1/dynamics_angle.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_1/dynamics_angle.jpg" width="90%"/></td>
  </tr>
</table>

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_1/dynamics_traj.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_1/dynamics_traj.jpg" width="90%"/></td>
  </tr>
</table>

### Dynamics - Open-Loop control 2m radius turn
<!-- TEST2_TABLE_START -->
| sim_type   | odom turn radius(m)   |   gt turn radius(m) | odom heading change(rad)   |   gt heading change(rad): |   sim time change(s) |   wall time change(s) |
|:-----------|:----------------------|--------------------:|:---------------------------|--------------------------:|---------------------:|----------------------:|
| gazebo     | 2.0                   |                1.97 | 2.49                       |                      2.42 |                10.04 |                 14.68 |
| isaacsim   | -                     |              298.07 | -                          |                      0.02 |                10.02 |                 24.37 |
<!-- TEST2_TABLE_END -->

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_2/dynamics_angle.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_2/dynamics_angle.jpg" width="90%"/></td>
  </tr>
</table>

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_2/dynamics_traj.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_2/dynamics_traj.jpg" width="90%"/></td>
  </tr>
</table>

### Dynamics - Open-Loop control 4m radius turn
<!-- TEST3_TABLE_START -->
| sim_type   | odom turn radius(m)   |   gt turn radius(m) | odom heading change(rad)   |   gt heading change(rad): |   sim time change(s) |   wall time change(s) |
|:-----------|:----------------------|--------------------:|:---------------------------|--------------------------:|---------------------:|----------------------:|
| gazebo     | 4.0                   |                4.46 | 1.25                       |                       1.2 |                10    |                 14.92 |
| isaacsim   | -                     |             1248.97 | -                          |                       0   |                 9.97 |                 24.33 |
<!-- TEST3_TABLE_END -->

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_3/dynamics_angle.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_3/dynamics_angle.jpg" width="90%"/></td>
  </tr>
</table>

<table>
  <tr>
    <td align="center"><b>Gazebo</b></td>
    <td align="center"><b>IsaacSim</b></td>
  </tr>
  <tr>
    <td><img src="sim_check/gazebo/test_3/dynamics_traj.jpg" width="90%"/></td>
    <td><img src="sim_check/isaac/test_3/dynamics_traj.jpg" width="90%"/></td>
  </tr>
</table>

### Gazebo Sensor and data publish rates

<!-- GAZEBO_DATA_RATE_TABLE_START -->
| topic                        |   count |   wall Hz |   sim Hz |
|:-----------------------------|--------:|----------:|---------:|
| /imu                         |     399 |     13.5  |    20.05 |
| /d435_rgb_camera/image_raw   |     605 |     20.45 |    30.35 |
| /d435_depth_camera/image_raw |     606 |     20.45 |    30.35 |
| /scan                        |     200 |      6.77 |    10.05 |
| /odom                        |    1995 |     67.4  |   100.05 |
| /clock                       |   19944 |    672.95 |  1000.05 |
| /tf                          |    2393 |     80.75 |   120.05 |
| /gt_pose                     |    1460 |     49.29 |    73.27 |
<!-- GAZEBO_DATA_RATE_TABLE_END -->

### IsaacSim Sensor and data publish rates
<!-- ISAAC_DATA_RATE_TABLE_START -->
| topic               |   count |   wall Hz |   sim Hz |
|:--------------------|--------:|----------:|---------:|
| /imu                |     398 |     49.83 |   120.61 |
| /rgb_camera/rgb     |     200 |     24.95 |    60.3  |
| /depth_camera/depth |     200 |     24.99 |    60.3  |
| /scan               |     398 |     49.84 |   120.61 |
| /odom               |     398 |     49.84 |   120.61 |
| /clock              |     398 |     49.84 |   120.61 |
| /tf                 |    1192 |    150.4  |   361.21 |
<!-- ISAAC_DATA_RATE_TABLE_END -->
