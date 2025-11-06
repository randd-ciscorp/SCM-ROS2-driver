// Copyright 2025 CIS Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CIS_SCM__TOF_DRIVER_HPP_
#define CIS_SCM__TOF_DRIVER_HPP_

#include <vector>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "cis_scm/ExternalDevice.hpp"

#define MAX_DEPTH 7.5

namespace cis_scm
{

/**
 * @brief Structure containing 3D coordinate arrays.
 */
struct XYZData
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
};

/**
 * @brief ROS2 node for managing SCM-ToF1 (and SCM-RGBD1) device.
 *
 * Handles device initialization, frame acquisition, depth processing,
 * and publishing of point cloud and depth image topics.
 */
class ToFNode : public rclcpp::Node
{
  public:
    /**
   * @brief Construct a new ToF node.
   * @param node_name ROS node name.
   * @param node_options Node options.
   */
    ToFNode(const std::string node_name, const rclcpp::NodeOptions & node_options);

    /**
   * @brief Start the ToF capture and publishing loop.
   */
    virtual void start();

  protected:
    int width_;
    int height_;

    cv::Mat depthMap_;

    sensor_msgs::msg::PointCloud2 ptcMsg_;
    sensor_msgs::msg::Image imgMsg_;
    XYZData xyzData_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<ExternalDevice> cap_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> depthPCLPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> depthImgPub_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoDepthPub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_depth_;

    rclcpp::CallbackGroup::SharedPtr crit_cb_grp_;
    rclcpp::CallbackGroup::SharedPtr non_crit_cb_grp_;

    std::string cameraDepthFrame_ = "camera_depth_frame";
    std::string topicDepthPrefix_ = "depth/";

    /**
   * @brief Initialize the ToF capture device.
   * @return 0 on success, negative on failure.
   */
    virtual int initCap();

    void importParams();
    XYZData splitXYZ(float * data);
    void dispInfo(DevInfo devInfo) const;
    void pubDepthImage(float * data);
    void pubDepthPtc(XYZData & data);

    void depthCallback();
    void infoCallback();
};
}  // namespace cis_scm
#endif  // CIS_SCM__TOF_DRIVER_HPP_
