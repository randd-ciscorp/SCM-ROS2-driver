// Copyright 2025 CIS Corporation
#include "cis_scm/rgbd_driver.hpp"

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifndef INTERNAL_DRIVER
#include "cis_scm/Device.h"
#else
#include "cis_scm/InternalDevice.hpp"
#endif

using namespace std::chrono_literals;

namespace cis_scm
{
RGBDNode::RGBDNode(const std::string node_name, const rclcpp::NodeOptions & node_options)
: ToFCVNode(node_name, node_options)
{
    rgbImgPub_ = create_publisher<sensor_msgs::msg::Image>(topicPrefix_ + "/img_rgb", 10);
    infoRGBPub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(topicPrefix_ + "/cam_rgb_info", 10);
    cinfo_rgb_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "scm");

    importRGBDParameters();
}

int RGBDNode::initCap()
{
#ifndef INTERNAL_DRIVER
    RCLCPP_INFO(get_logger(), "EXTERNAL DRIVERRRR");
    cap_ = std::make_unique<ExternalDevice>();
    if (!cap_->connect(480, 640))
#else
    RCLCPP_INFO(get_logger(), "INTERNAL DRIVERRRR");
    cap_ = std::make_unique<internal::RGBDInternalDevice>(
        this->get_parameter("tof_params_path").as_string(), this->get_parameter("align").as_bool());
    if (!cap_->connect())
#endif
    {
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
    header.frame_id = "camera";
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
    header.frame_id = "cam_depth";
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
#ifdef INTERNAL_DRIVER
    depthPCLPub_.publish(ptcMsg_);
#else
    depthPCLPub_->publish(ptcMsg_);
#endif
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
        infoMsg->header.frame_id = "cam_depth";
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
