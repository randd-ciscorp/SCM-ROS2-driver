#include "cis_scm/tof_driver.hpp"

#include <string>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>

#ifndef INTERNAL_DRIVER
#include "cis_scm/Device.h"
#else
#include "cis_scm/InternalDevice.hpp"
#endif

using namespace std::chrono_literals;

namespace cis_scm
{

ToFCVNode::ToFCVNode(const std::string node_name, const rclcpp::NodeOptions & node_options) : Node(node_name, node_options){
    depthImgPub_ = create_publisher<sensor_msgs::msg::Image>(topicPrefix_ + "/img_depth", 10);
    depthPCLPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topicPrefix_ + "/pcl_depth", 10);
    infoPub_ = create_publisher<sensor_msgs::msg::CameraInfo>(topicPrefix_ + "/cam_info", 10);

    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "scm");
    importParams();

}

int ToFCVNode::initCap()
{
#ifndef INTERNAL_DRIVER
    RCLCPP_INFO(get_logger(),"EXTERNAL DRIVERRRR");
    cap_ = std::make_unique<ExternalDevice>();
    if (!cap_->connect(480, 640))
#else
    RCLCPP_INFO(get_logger(),"INTERNAL DRIVERRRR");
    cap_ = std::make_unique<internal::ToFInternalDevice>(this->get_parameter("tof_params_path").as_string());
    if (!cap_->connect())
#endif
    {
        RCLCPP_INFO(get_logger(), "Camera connected");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Camera connection failed");
    }
    return 0;
}

void ToFCVNode::dispInfo(DevInfo devInfo) const{
    RCLCPP_INFO(get_logger(), "Device name: %s", devInfo.devName.c_str());
    RCLCPP_INFO(get_logger(), "Driver version: %s", devInfo.driverVers.c_str());
    RCLCPP_INFO(get_logger(), "Serial Number: %s", devInfo.sn.c_str());
    RCLCPP_INFO(get_logger(), "Width: %d", devInfo.width);
    RCLCPP_INFO(get_logger(), "Height: %d", devInfo.height);
}

void ToFCVNode::importParams(){
    // Cam params
    if(!this->has_parameter("camera_params")){
        this->declare_parameter("camera_params", "package://cis_scm/cam_param.yaml");
    }
    std::string param_file_path = this->get_parameter("camera_params").as_string();

    if (param_file_path != "")
    {
        RCLCPP_INFO(get_logger(), "Load parameters file: %s", param_file_path.c_str());
        if (cinfo_->validateURL(param_file_path))
        {
            cinfo_->loadCameraInfo(param_file_path);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not find parameter file at: %s", param_file_path.c_str());
            rclcpp::shutdown();
        }
    }

    // ToF params
    if (!this->has_parameter("tof_params_path"))
    {
        // Default path in SCM-ToF1, probably need to change
        this->declare_parameter("tof_params_path", "/home/root/tof_params");
    }
}

void ToFCVNode::start(){
    if(initCap())
    {
        RCLCPP_ERROR(get_logger(), "Camera initialization failed");
        rclcpp::shutdown();
    }

    cap_->isConnected();
    if (cap_->isConnected())
    {
        DevInfo devInfo = cap_->getInfo();
        dispInfo(devInfo);
        width_ = devInfo.width;
        height_ = devInfo.height;

        // Formating pointcloud message
        ptcMsg_.width = width_;
        ptcMsg_.height = height_;
        ptcMsg_.is_bigendian = true;
        sensor_msgs::PointCloud2Modifier ptcModif(ptcMsg_);
        ptcModif.setPointCloud2FieldsByString(1, "xyz");
        ptcModif.resize(ptcMsg_.width * ptcMsg_.height);

        RCLCPP_INFO(get_logger(), "Start capturing");
        timer_ = this->create_wall_timer(30ms, std::bind(&ToFCVNode::depthCallback, this));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Camera connection failed");
        rclcpp::shutdown();
    }
}

XYZData ToFCVNode::splitXYZ(float* data){
    int size = width_ * height_;
    XYZData output;
    output.x.resize(width_ * height_);
    output.y.resize(width_ * height_);
    output.z.resize(width_ * height_);
    for (int i = 0, j = 0; j < size; i+=3, j++)
    {
        output.x[j] = data[i]; // X
        output.y[j] = data[i+1]; // Y
        output.z[j] = data[i+2]; // Z
    }
    return output;
}

void ToFCVNode::pubDepthImage(float* data){
    greyDepth_ = cv::Mat(height_, width_, CV_32FC1, data);
    greyDepth_.convertTo(greyDepth_, CV_8UC1, 255./ MAX_DEPTH);

    // Grey -> Hue
    hueDepth_ = cv::Mat(height_, width_, CV_8UC3);
    cv::applyColorMap(greyDepth_, hueDepth_, cv::COLORMAP_JET);
    if(!hueDepth_.isContinuous()){
        printf("NOT CONTINOUS");
    }

    // Msg header
    auto header = std_msgs::msg::Header();
    header.frame_id = "camera";
    header.stamp = this->get_clock()->now();

    // 2D Image publishing
    imgMsg_ = sensor_msgs::msg::Image();
    imgMsg_.header = header;
    imgMsg_.width = width_;
    imgMsg_.height = height_;
    imgMsg_.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    imgMsg_.step = width_ * sizeof(uchar)*3;
    imgMsg_.is_bigendian = true;
    imgMsg_.data.assign(hueDepth_.data, hueDepth_.data + hueDepth_.rows*hueDepth_.cols*3);
    depthImgPub_->publish(std::move(imgMsg_));
}

void ToFCVNode::pubDepthPtc(XYZData &data){
        // Header
        auto header = std_msgs::msg::Header();
        rclcpp::Time time;
        header.stamp = this->get_clock()->now();
        header.frame_id = "cam_depth";
        ptcMsg_.header = header;

        sensor_msgs::PointCloud2Iterator<float> iter_x(ptcMsg_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ptcMsg_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ptcMsg_, "z");

        //fill data
        for (size_t i = 0; i < ptcMsg_.height * ptcMsg_.width; i++, ++iter_x, ++iter_y, ++iter_z)
        {
            float z = data.z[i];
            if (z <= 0)
            {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
            }
            else{
                *iter_x = data.x[i];
                *iter_y = data.y[i];
                *iter_z = z;
            }
        }

        depthPCLPub_->publish(ptcMsg_);
}

void ToFCVNode::depthCallback(){
    // Init 2D Mats

    std::vector<float> frameData = std::vector<float>(width_ * height_ * 3);
    while (rclcpp::ok() && cap_->isConnected())
    {
        if(cap_->getData(reinterpret_cast<uint8_t*>(frameData.data()))){
            cap_->disconnect();
            break;
        }

        // Cam Info
        auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
        infoMsg->header.frame_id = "cam_depth";
        infoMsg->header.stamp = this->get_clock()->now();
        infoPub_->publish(*infoMsg);

        // 2D Image
        xyzData_ = splitXYZ(frameData.data());
        pubDepthImage(xyzData_.z.data());

        // 3D Image
        pubDepthPtc(xyzData_);
    }

    if (!cap_->isConnected())
    {
        RCLCPP_ERROR(get_logger(), "Camera connection lost or unavailable");
        rclcpp::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(get_logger(), "New camera connection attempt");
    }
}
}
