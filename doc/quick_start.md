
# Quick Start


## ROS 2 installation
If it is not already installed, please install the right ROS 2 dristribution

[ROS 2 Humble Installation Tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

## ROS 2 Workspace
If not created yet, create your workspace
``` bash
$ mkdir ros_ws & cd ros_ws
```

## Dependencies
The following packages are necessary
``` bash
$ sudo apt update
$ sudo apt install ros-humble-camera-info-manager
```

## Driver package building
``` bash
$ git clone https://gitlab3.cis.local/imx/scm-ros2-driver.git ./src/
$ source /opt/ros/humble/setup.bash
$ colcon build
```

## Camera setup
1. Connect the USB3.0 cable from host PC/Robot to the camera
2. Connect the external power source to the camera
- **NOTE**: The camera will take a few seconds to boot

## ROS driver node launching

``` bash
$ source install/setup.bash 
``` 

### SCM-ToF1
#### ToF + Display
``` bash
$ ros2 launch cis_scm tof_viz_launch.py
```

#### ToF (no display)
``` bash
$ ros2 launch cis_scm tof_launch.py
```

***

### SCM-RGBD1
#### RGBD + Display
``` bash
$ ros2 launch cis_scm rgbd_viz_launch.py
```

#### RGBD (no display)
``` bash
$ ros2 launch cis_scm rgbd_launch.py
```

***

### SCM-2M1 / SCM-8M1
#### RGB + Display
``` bash
$ ros2 launch cis_scm rgb_viz_launch.py
```

#### RGB (no display)
``` bash
$ ros2 launch cis_scm rgb_launch.py
```

## Stop node

Press `Ctrl + C` to stop the driver node.
