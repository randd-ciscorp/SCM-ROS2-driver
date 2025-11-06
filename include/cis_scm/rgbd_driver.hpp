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

#ifndef CIS_SCM__RGBD_DRIVER_HPP_
#define CIS_SCM__RGBD_DRIVER_HPP_

#include <vector>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "cis_scm/tof_driver.hpp"

namespace cis_scm
{

struct XYZRGBData
{
    XYZData xyz;
    std::vector<cv::Vec3b> rgb;
    std::vector<uint8_t> ir;
};

struct XYZRGBPixel
{
    float x;
    float y;
    float z;
    cv::Vec3b rgb;
    uint8_t a;
};

class RGBDNode : public ToFNode
{
  public:
    RGBDNode(const std::string node_name, const rclcpp::NodeOptions & node_options);

    void start() override;

  protected:
    std::vector<uint8_t> frameData_;

    sensor_msgs::msg::Image imgRGBMsg_;
    bool isPCLNoColor_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> rgbImgPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoRGBPub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_rgb_;

    std::string cameraColorFrame_ = "camera_color_frame";
    std::string topicRGBPrefix_ = "aligned_color_to_depth/";

    int initCap() override;
    void importRGBDParameters();

    XYZRGBData splitXYZRGBData(uint8_t * xyzrgbData);

    void pubRGBImage(uint8_t * data);
    void pubRGBDPtc(XYZRGBData & xyzrgbdata);

    void RGBDCallback();
};
}  // namespace cis_scm
#endif  // CIS_SCM__RGBD_DRIVER_HPP_
