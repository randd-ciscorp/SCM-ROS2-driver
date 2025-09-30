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

#include "cis_scm/rgb_driver.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifndef INTERNAL_DRIVER
#include "cis_scm/ExternalDevice.hpp"
#else
#include <image_transport/image_transport.hpp>

#include "cis_scm/InternalDevice.hpp"
#endif

using namespace std::literals::chrono_literals;
namespace cis_scm
{

RGBNode::RGBNode(const std::string node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options)
{
    infoPub_ = create_publisher<sensor_msgs::msg::CameraInfo>(topicPrefix_ + "/cam_info", 10);

    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "scm");

#ifndef INTERNAL_DRIVER
    imgPub_ = create_publisher<sensor_msgs::msg::Image>(topicPrefix_ + "/img_rgb", 10);
#endif
    importParams();
}

void RGBNode::importParams()
{
    // Cam params
    if (!this->has_parameter("camera_params")) {
        this->declare_parameter("camera_params", "package://cis_scm/cam_param.yaml");
    }
    std::string param_file_path = this->get_parameter("camera_params").as_string();

    if (param_file_path != "") {
        RCLCPP_INFO(get_logger(), "Load parameters file: %s", param_file_path.c_str());
        if (cinfo_->validateURL(param_file_path)) {
            cinfo_->loadCameraInfo(param_file_path);
        } else {
            RCLCPP_ERROR(
                get_logger(), "Could not find parameter file at: %s", param_file_path.c_str());
            rclcpp::shutdown();
        }
    }

    // SCM-2M1 or SCM-8M1
    if (!this->has_parameter("rgb_device")) {
        this->declare_parameter("rgb_device", "2m1");
    }
    if (this->get_parameter("rgb_device").as_string() == "2m1") {
        width_ = 1920;
        height_ = 1080;
    } else if (this->get_parameter("rgb_device").as_string() == "8m1") {
        width_ = 3840;
        height_ = 2160;
    } else {
        RCLCPP_ERROR(
            get_logger(),
            "ERROR: rgb_device parameter has a wrong value. Please choose between '2m1' or '8m1'.");
    }
}

#ifdef INTERNAL_DRIVER
void RGBNode::initImageTransport()
{
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    imgPub_ = it_->advertise(topicPrefix_ + "/image_raw", 10);
}
#endif

int RGBNode::initCap()
{
#ifdef INTERNAL_DRIVER
    cap_ = std::make_unique<internal::RGBInternalDevice>(new cis::CameraAR0234());
    if (!cap_->connect())
#else
    cap_ = std::make_unique<ExternalDevice>();
    if (!cap_->connect(width_, height_))
#endif
    {
        RCLCPP_INFO(get_logger(), "Camera connected");
    } else {
        RCLCPP_ERROR(get_logger(), "Camera connection failed");
    }
    return 0;
}

void RGBNode::dispInfo(DevInfo devInfo) const
{
    RCLCPP_INFO(get_logger(), "Device name: %s", devInfo.devName.c_str());
    RCLCPP_INFO(get_logger(), "Driver version: %s", devInfo.driverVers.c_str());
    RCLCPP_INFO(get_logger(), "Serial Number: %s", devInfo.sn.c_str());
    RCLCPP_INFO(get_logger(), "Width: %d", devInfo.width);
    RCLCPP_INFO(get_logger(), "Height: %d", devInfo.height);
}

void RGBNode::start()
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

        RCLCPP_INFO(get_logger(), "Start capturing");
        timer_ = this->create_wall_timer(33ms, std::bind(&RGBNode::imgCallback, this));
    } else {
        RCLCPP_ERROR(get_logger(), "Camera connection failed");
        rclcpp::shutdown();
    }
}

void RGBNode::pubImage(uint8_t * data)
{
    // Msg header
    auto header = std_msgs::msg::Header();
    header.frame_id = "camera";
    header.stamp = this->get_clock()->now();

    // RGB image publishing
    imgMsg_ = sensor_msgs::msg::Image();
    imgMsg_.header = header;
    imgMsg_.width = width_;
    imgMsg_.height = height_;
    imgMsg_.encoding = sensor_msgs::image_encodings::BGR8;
    imgMsg_.step = width_ * 3;
    imgMsg_.is_bigendian = true;
    imgMsg_.data.assign(data, data + imgMsg_.step * imgMsg_.height);
#ifdef INTERNAL_DRIVER
    imgPub_.publish(std::move(imgMsg_));
#else
    imgPub_->publish(std::move(imgMsg_));
#endif
}

void RGBNode::imgCallback()
{
    auto st = this->now();
    if (!cap_->isConnected()) {
        RCLCPP_ERROR(get_logger(), "Camera connection lost or unavailable");
        rclcpp::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(get_logger(), "New camera connection attempt");
    } else {
        cv::Mat imgData(height_, width_, CV_8UC2);
        if (!cap_->getData(imgData.data)) {
            cv::resize(imgData, imgData, cv::Size(width_, height_));
            // Cam Info
            auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
            infoMsg->header.frame_id = "cam_depth";
            infoMsg->header.stamp = this->get_clock()->now();
            infoPub_->publish(*infoMsg);

            // 2D image
            cv::cvtColor(imgData, imgData, cv::COLOR_YUV2BGR_YUYV);
            pubImage(imgData.data);
        }
    }
    auto sp = this->now();
}

}  // namespace cis_scm
