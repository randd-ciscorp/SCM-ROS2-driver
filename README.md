# SCM ROS 2 driver

[![Main pipeline status](https://gitlab3.cis.local/imx/scm-ros2-driver/badges/main/pipeline.svg)](https://gitlab3.cis.local/imx/scm-ros2-driver/-/commits/main)

ROS 2 driver node for CIS SCM camera series.

## Required system configurations

- Host Linux PC/Robot
    - Supported OS / ROS distro
        - Ubuntu 22.04 / ROS 2 Humble
    - USB3.0 port

- SCM camera
- USB3.0 / micro USB type-B cable
- Power supply (12V 5A)

## Topics

### SCM-ToF1
- `depth/camera_info` --> Camera info (CameraInfo)
- `depth/points` --> 3D point cloud (Pointcloud2)
- `depth/image` --> 2D Depth map (Image)

### SCM-RGBD1
- `aligned_color_to_depth/camera_info` --> Aligned RGB camera info (CameraInfo)
- `aligned_color_to_depth/image` --> Aligned RGB to depth data (Image)
- `depth/camera_info` --> Depth camera info (CameraInfo)
- `depth/image` --> 2D Depth map (Image)
- `depth/points` --> 3D Colored point cloud (Pointcloud2)

### SCM-2M1 / SCM-8M1
- `color/camera_info` --> RGB camera info (CameraInfo)
- `color/image` --> RGB image (Image)

## Quick Start


### ROS 2 installation
If it is not already installed, please install the right ROS 2 dristribution

[ROS 2 Humble Installation Tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### ROS 2 Workspace
If not created yet, create your workspace
``` bash
$ mkdir ros_ws & cd ros_ws
```

### Dependencies
The following packages are necessary
``` bash
$ sudo apt update
$ sudo apt install ros-humble-camera-info-manager
```

### Driver package building
``` bash
$ git clone https://gitlab3.cis.local/imx/scm-ros2-driver.git ./src/
$ source /opt/ros/humble/setup.bash
$ colcon build
```

### Camera setup
1. Connect the USB3.0 cable from host PC/Robot to the camera
2. Connect the external power source to the camera
- **NOTE**: The camera will take a few seconds to boot

### ROS driver node launching

``` bash
$ source install/setup.bash 
``` 

#### SCM-ToF1
##### ToF + Display
``` bash
$ ros2 launch cis_scm tof_viz_launch.py
```

##### ToF (no display)
``` bash
$ ros2 launch cis_scm tof_launch.py
```

***

#### SCM-RGBD1
##### RGBD + Display
``` bash
$ ros2 launch cis_scm rgbd_viz_launch.py
```

##### RGBD (no display)
``` bash
$ ros2 launch cis_scm rgbd_launch.py
```

***

#### SCM-2M1 / SCM-8M1
##### RGB + Display
``` bash
$ ros2 launch cis_scm rgb_viz_launch.py
```

##### RGB (no display)
``` bash
$ ros2 launch cis_scm rgb_launch.py
```

### Stop node

Press `Ctrl + C` to stop the driver node.

## Contact
`@leoboule Léo Boulé, <leoboule@ciscorp.co.jp>` (Author)
