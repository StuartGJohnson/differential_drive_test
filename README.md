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
| gazebo     | 0                     |                0.04 | 5.03                       |                      5.08 |                10.03 |                 14.75 |
| isaacsim   | -                     |                0.08 | -                          |                      0.96 |                10.03 |                 24.76 |
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
| gazebo     | 1.0                   |                1    | 5.06                       |                      5.04 |                10.07 |                  14.8 |
| isaacsim   | -                     |               45.99 | -                          |                      0.12 |                 9.98 |                  24.3 |
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
| gazebo     | 2.0                   |                1.97 | 2.51                       |                      2.45 |                10.03 |                 14.73 |
| isaacsim   | -                     |              297.99 | -                          |                      0.02 |                 9.93 |                 24.43 |
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
| gazebo     | 4.0                   |                4.46 | 1.25                       |                       1.2 |                10.02 |                 14.81 |
| isaacsim   | -                     |             2197.49 | -                          |                       0   |                10.07 |                 24.46 |
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
| /imu                         |     400 |     13.34 |    20.05 |
| /d435_rgb_camera/image_raw   |     604 |     20.12 |    30.35 |
| /d435_depth_camera/image_raw |     604 |     20.11 |    30.35 |
| /scan                        |     200 |      6.66 |    10.05 |
| /odom                        |    1999 |     66.63 |   100.05 |
| /clock                       |   19930 |    662.7  |   999.85 |
| /tf                          |    2401 |     79.78 |   119.75 |
| /gt_pose                     |    1488 |     49.42 |    74.19 |
<!-- GAZEBO_DATA_RATE_TABLE_END -->

### IsaacSim Sensor and data publish rates
<!-- ISAAC_DATA_RATE_TABLE_START -->
| topic               |   count |   wall Hz |   sim Hz |
|:--------------------|--------:|----------:|---------:|
| /imu                |     400 |     49.83 |   120.6  |
| /rgb_camera/rgb     |     200 |     24.97 |    60.3  |
| /depth_camera/depth |     200 |     25.07 |    60.3  |
| /scan               |     400 |     49.83 |   120.6  |
| /odom               |     400 |     49.83 |   120.6  |
| /clock              |     400 |     49.83 |   120.6  |
| /tf                 |    1194 |    149.45 |   361.82 |
<!-- ISAAC_DATA_RATE_TABLE_END -->
