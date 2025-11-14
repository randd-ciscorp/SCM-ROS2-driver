
# Quick Start


## ROS 2 installation
If it is not already installed, please install a ROS 2 distribution  
- [ROS 2 Humble Installation Tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Jazzy Installation Tutorial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Kilted Installation Tutorial](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Rolling Installation Tutorial](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html)


## `SCM-ROS2-driver` Installation
1. If not created yet, create your workspace, and add `SCM-ROS2-driver` in it
    ``` bash
    $ mkdir ros_ws && cd ros_ws
    $ git clone https://github.com/randd-ciscorp/SCM-ROS2-driver.git ./src/
    ```

2. Source ROS environment
    ``` bash
    $ source /opt/ros/<ROS_DRISTRO>/setup.bash
    ```

3. Install dependencies
    ```bash
    $ sudo apt update
    $ rosdep install --from-paths src -y
    ```

4. Build
    ```bash
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
