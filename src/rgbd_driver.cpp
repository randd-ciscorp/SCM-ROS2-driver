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

#include "cis_scm/rgbd_driver.hpp"

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "cis_scm/Device.h"

using namespace std::chrono_literals;

namespace cis_scm
{
RGBDNode::RGBDNode(const std::string node_name, const rclcpp::NodeOptions & node_options)
: ToFNode(node_name, node_options)
{
    rgbImgPub_ = create_publisher<sensor_msgs::msg::Image>(topicRGBPrefix_ + "image", 10);
    infoRGBPub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(topicRGBPrefix_ + "camera_info", 10);
    cinfo_rgb_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "scm-rgbd1");
    cinfo_depth_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "scm-tof1");
    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "scm-rgbd1");

    importRGBDParameters();
}

int RGBDNode::initCap()
{
    cap_ = std::make_unique<ExternalDevice>();
    if (!cap_->connect(480, 640)) {
        RCLCPP_INFO(get_logger(), "Camera connected");
    } else {
        RCLCPP_ERROR(get_logger(), "Camera connection failed");
    }

    return 0;
}

void RGBDNode::importRGBDParameters()
{
    // Check if pointcloud color is not needed
    if (!this->has_parameter("pointcloud_rgb")) {
        this->declare_parameter("pointcloud_rgb", true);
    }
    RCLCPP_INFO(get_logger(), "Getting parameter");
    isPCLNoColor_ = !this->get_parameter("pointcloud_rgb").as_bool();

    // Check if rgb should be aligned to depth
    if (!this->has_parameter("align")) {
        this->declare_parameter("align", true);
    }

    // Cam params

    if (!this->has_parameter("tof_camera_params")) {
        this->declare_parameter(
            "tof_camera_params", "package://cis_scm/config/cam_params/tof_params.yaml");
    }
    std::string tof_param_file_path = this->get_parameter("tof_camera_params").as_string();
    if (tof_param_file_path != "") {
        RCLCPP_INFO(get_logger(), "Load parameters file: %s", tof_param_file_path.c_str());
        if (cinfo_->validateURL(tof_param_file_path)) {
            cinfo_->loadCameraInfo(tof_param_file_path);
            cinfo_depth_->loadCameraInfo(tof_param_file_path);
        } else {
            RCLCPP_ERROR(
                get_logger(), "Could not find parameter file at: %s", tof_param_file_path.c_str());
            rclcpp::shutdown();
        }
    }

    if (!this->has_parameter("rgb_camera_params")) {
        this->declare_parameter(
            "rgb_camera_params", "package://cis_scm/config/cam_params/rgbd_color_params.yaml");
    }
    std::string rgb_param_file_path = this->get_parameter("rgb_camera_params").as_string();
    if (rgb_param_file_path != "") {
        RCLCPP_INFO(get_logger(), "Load parameters file: %s", rgb_param_file_path.c_str());
        if (cinfo_rgb_->validateURL(rgb_param_file_path)) {
            cinfo_rgb_->loadCameraInfo(rgb_param_file_path);
        } else {
            RCLCPP_ERROR(
                get_logger(), "Could not find parameter file at: %s", rgb_param_file_path.c_str());
            rclcpp::shutdown();
        }
    }
}

void RGBDNode::start()
{
    if (initCap()) {
        RCLCPP_ERROR(get_logger(), "Camera initialization failed");
        rclcpp::shutdown();
    }

    if (cap_->isConnected()) {
        DevInfo devInfo = cap_->getInfo();
        dispInfo(devInfo);
        width_ = devInfo.width;
        height_ = devInfo.height;

        // Formating pointcloud message
        ptcMsg_.width = width_;
        ptcMsg_.height = height_;
        ptcMsg_.is_bigendian = true;
        sensor_msgs::PointCloud2Modifier ptcModif(ptcMsg_);
        if (isPCLNoColor_) {
            ptcModif.setPointCloud2FieldsByString(1, "xyz");
        } else {
            ptcModif.setPointCloud2FieldsByString(2, "xyz", "rgb");
        }
        ptcModif.resize(ptcMsg_.width * ptcMsg_.height);

        // Start capturing
        RCLCPP_INFO(get_logger(), "Start Capturing");
        timer_ = this->create_wall_timer(30ms, std::bind(&RGBDNode::RGBDCallback, this));
    } else {
        RCLCPP_ERROR(get_logger(), "Camera not connected");
        rclcpp::shutdown();
    }
}

XYZRGBData RGBDNode::splitXYZRGBData(uint8_t * xyzrgbData)
{
    int size = width_ * height_;
    XYZRGBPixel pix;

    XYZRGBData output;
    output.xyz.x.resize(width_ * height_);
    output.xyz.y.resize(width_ * height_);
    output.xyz.z.resize(width_ * height_);
    output.rgb.resize(width_ * height_);

    XYZRGBPixel * pixelPtr = reinterpret_cast<XYZRGBPixel *>(xyzrgbData);
    for (int i = 0; i < size; i++) {
        output.xyz.x[i] = pixelPtr[i].x;
        output.xyz.y[i] = pixelPtr[i].y;
        output.xyz.z[i] = pixelPtr[i].z;
        output.rgb[i] = pixelPtr[i].rgb;
    }
    return output;
}

void RGBDNode::pubRGBImage(uint8_t * rgbdata)
{
    // Msg header
    auto header = std_msgs::msg::Header();
    header.frame_id = cameraColorFrame_;
    header.stamp = this->get_clock()->now();

    // 2D Image publishing
    imgRGBMsg_ = sensor_msgs::msg::Image();
    imgRGBMsg_.header = header;
    imgRGBMsg_.width = width_;
    imgRGBMsg_.height = height_;
    imgRGBMsg_.encoding = sensor_msgs::image_encodings::BGR8;
    imgRGBMsg_.step = width_ * sizeof(uint8_t) * 3;
    imgRGBMsg_.is_bigendian = true;
    imgRGBMsg_.data.assign(rgbdata, rgbdata + imgRGBMsg_.height * imgRGBMsg_.step);
    rgbImgPub_->publish(std::move(imgRGBMsg_));
}

void RGBDNode::pubRGBDPtc(XYZRGBData & xyzrgbData)
{
    // Header
    auto header = std_msgs::msg::Header();
    rclcpp::Time time;
    header.stamp = this->get_clock()->now();
    header.frame_id = cameraDepthFrame_;
    ptcMsg_.header = header;
    sensor_msgs::PointCloud2Iterator<float> iter_x(ptcMsg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(ptcMsg_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(ptcMsg_, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(ptcMsg_, "rgb");

    for (size_t i = 0; i < ptcMsg_.width * ptcMsg_.height;
         ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
        float z = xyzrgbData.xyz.z[i];

        bool isRGBNull = xyzrgbData.rgb[i] == cv::Vec3b(0, 0, 0);

        if (z <= 0 || isRGBNull) {
            *iter_x = std::numeric_limits<float>::quiet_NaN();
            *iter_y = std::numeric_limits<float>::quiet_NaN();
            *iter_z = std::numeric_limits<float>::quiet_NaN();
            *iter_rgb = 0;
        } else {
            *iter_x = xyzrgbData.xyz.x[i];
            *iter_y = xyzrgbData.xyz.y[i];
            *iter_z = xyzrgbData.xyz.z[i];
            iter_rgb[0] = xyzrgbData.rgb[i][0];
            iter_rgb[1] = xyzrgbData.rgb[i][1];
            iter_rgb[2] = xyzrgbData.rgb[i][2];
        }
    }
    depthPCLPub_->publish(ptcMsg_);
}

void RGBDNode::RGBDCallback()
{
    frameData_ = std::vector<uint8_t>(width_ * height_ * sizeof(XYZRGBPixel));
    XYZRGBData xyzrgbData;
    xyzrgbData.xyz.x = std::vector<float>(width_ * height_);
    xyzrgbData.xyz.y = std::vector<float>(width_ * height_);
    xyzrgbData.xyz.z = std::vector<float>(width_ * height_);
    xyzrgbData.rgb = std::vector<cv::Vec3b>(width_ * height_);
    xyzrgbData.ir = std::vector<uint8_t>(width_ * height_);

    while (rclcpp::ok() && cap_->isConnected()) {
        if (cap_->getData(frameData_.data()) < 0) {
            cap_->disconnect();
            break;
        }

        // Cam Info
        auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
        infoMsg->header.frame_id = cameraDepthFrame_;
        infoMsg->header.stamp = this->get_clock()->now();
        infoPub_->publish(*infoMsg);

        xyzrgbData = splitXYZRGBData(frameData_.data());

        pubDepthImage(xyzrgbData.xyz.z.data());
        pubRGBImage(reinterpret_cast<uint8_t *>(xyzrgbData.rgb.data()));
        if (isPCLNoColor_) {
            pubDepthPtc(xyzrgbData.xyz);
        } else {
            pubRGBDPtc(xyzrgbData);
        }
    }
}
}  // namespace cis_scm
