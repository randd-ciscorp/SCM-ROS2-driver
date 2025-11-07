# Changelog


## v0.3.1 2025-11-07
### Features
- [X] Added sample application with Point Cloud Library (2597811a)
- [X] Added documentation, compatible with rosdoc2 (d619d1b7)

### Updates | Fixes
- [X] Depth map published on `depth/image` topic is not converted to JETMAP color anymore. (75e293c3)
- [X] Split external and internal versions in two different branches. `main`` and `dev` branches have only the external one (87191ee0)


## v0.3.0 2025-10-20
### Features
- [X] SCM-2M1 & SCM-8M1 ROS 2 driver (5ccafb26)
- [X] Camera controls setting through ROS parameters and CIS protocol
    - [X] CIS protocol setter (70bf9751)
    - [X] CIS protocol getter (d56ec39d)
    - [X] SCM RGB ROS 2 parameters (0d19b90f)
    - [X] SCM-ToF1 ROS 2 parameters (4a998c8a)
    - [ ] SCM controls and ROS 2 parameters on internal driver (WiP)


- [X] Rviz configs for each camera (e3d2f994)
- [X] CI gitlab pipeline (246684c4)
- [X] Apache 2 License (1415c146)


### Updates | Fixes
- [X] Renamed topics and frames (5825e432) (f9746ce6)
- [X] All camera intrinsic & extinsic default parameters (9eabdb98)
- [ ] Internal driver (4e7c0f00)
    - [X] Topic publishing from the SCM
    - [ ] Messages transfer speed is slow (WiP)

## v0.2.1 2025-05-21
### Features
- [X] RGBD driver version
- [X] Option to display RGBD pointcloud without color
### Updates | Fixes
- [X] ToF more compatible with other modules like RGBD
- [X] ToF launch files --> Rewrote in python
- [X] Changed package name to "cis_scm"

## v0.2.0 2025-05-12
### Update | Fixes
- [X] Clean + Opti code TOF1 driver version

## v0.1.0 2024-05-xx
- [X] First tof1 & RGBD1 drivers
