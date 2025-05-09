# ToF1 ROS driver

ROS 2 driver node for CIS SCM1-ToF camera

## Required system configurations

- Host Linux PC
    - Supported OS / ROS distro
        - Ubuntu 22.04 / ROS 2 Humble
    - USB3.0 port

- SCM-ToF1
- USB3.0 / micro USB type-B cable
- Power supply (12V 5A)

## Topics
- `/camera/depth/cam_info` --> Camera info (CameraInfo)
- `/camera/depth/pcl_depth` --> 3D Depth Pointcloud (Pointcloud2)
- `/camera/depth/img_depth` --> 2D Depth map (Image)

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
$ git clone git@gitlab3.cis.local:leoboule/tof1-ros-driver.git ./src/
$ source /opt/ros/humble/setup.bash
$ colcon build
```

### Camera setup
1. Connect the USB3.0 cable from host PC/Robot to the camera
2. Connect the external power source to the camera
- **NOTE**: The camera will takes few seconds to boot

### ROS driver node launching

``` bash
$ source install/setup.bash 
``` 

#### ToF + Display
``` bash
$ ros2 launch tof1_driver tof_viz.launch
```


#### ToF (no display)
``` bash
$ ros2 launch tof1_driver tof.launch
```

### Stop node

Press `Ctrl + C` to stop the driver node.

## Contact
`@leoboule Léo Boulé, <leoboule@ciscorp.co.jp>` (Author)