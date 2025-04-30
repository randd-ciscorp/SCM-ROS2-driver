#include "tof1_driver/tof_driver.hpp"

#include <string>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

namespace tof_driver
{

ToFCVNode::ToFCVNode(const rclcpp::NodeOptions & node_options) : Node("tof_driver", node_options){
    imgPub_ = create_publisher<sensor_msgs::msg::Image>(topicPrefix_ + "/img_depth", 10);
    pclPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topicPrefix_ + "/pcl_depth", 10);
    infoPub_ = create_publisher<sensor_msgs::msg::CameraInfo>(topicPrefix_ + "/cam_info", 10);

    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "scm1-tof");
    importParams();

    cap_ = std::make_shared<Device>();
    cap_->connect(480, 640);
    DevInfo devInfo = cap_->getInfo();
    width_ = devInfo.width;
    height_ = devInfo.height;

    if (cap_->isConnected())
    {
        dispInfo(devInfo);
        this->start();
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Camera connection failed");
    }
}

void ToFCVNode::dispInfo(DevInfo devInfo){
    RCLCPP_INFO(get_logger(), "Connected to:");
    RCLCPP_INFO(get_logger(), "Device name: %s", devInfo.devName.c_str());
    RCLCPP_INFO(get_logger(), "Driver version: %s", devInfo.driverVers.c_str());
    RCLCPP_INFO(get_logger(), "Serial Number: %s", devInfo.sn.c_str());
    RCLCPP_INFO(get_logger(), "Width: %d", devInfo.width);
    RCLCPP_INFO(get_logger(), "Height: %d", devInfo.height);
}

void ToFCVNode::importParams(){
    if(!this->has_parameter("camera_params")){
        this->declare_parameter("camera_params", "package://tof1_driver/cam_param.yaml");
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
            RCLCPP_WARN(get_logger(), "Could not find the parameter file at: %s", param_file_path.c_str());
        }
    }
}

void ToFCVNode::start(){
    std::cout << "Start Cam callback" << std::endl;
    timer_ = this->create_wall_timer(30ms, std::bind(&ToFCVNode::depthCallback, this));
}

void ToFCVNode::infoCallback(){
    while(rclcpp::ok() && cap_->isConnected()){
        auto camInfo = sensor_msgs::msg::CameraInfo();
        camInfo.width = width_;
        camInfo.height = height_;
    }
}

std::vector<std::vector<float>> ToFCVNode::splitXYZ(float* data){
    int size = width_ * height_;
    std::vector<std::vector<float>> output(3, std::vector<float>(size, 0));
    for (int i = 0; i < size*3; i+=3)
    {
        output[0][i/3] = data[i]; // X
        output[1][i/3] = data[i+1]; // Y
        output[2][i/3] = data[i+2]; // Z
    }
    return output;
}

void ToFCVNode::normalize(float* vec){
    for (int i = 0; i < width_ * height_; i++)
    {
        //7.5m -> max distance
        vec[i] = vec[i] / 7.5 * 255.; 
    }
}

void ToFCVNode::pubDepthImage(float* data){
    xyzData_ = splitXYZ(data);
    
    normalize(xyzData_[2].data());
    std::vector<uchar> depth_u8 = std::vector<uchar>(xyzData_[2].size());
    for (size_t i = 0; i < xyzData_[2].size(); i++)
    {
        depth_u8[i] = (uchar)xyzData_[2][i];
    }

    // Grey -> Hue
    cv::Mat hueDepth(height_, width_, CV_8UC3);
    cv::applyColorMap(depth_u8, hueDepth, cv::COLORMAP_JET);
    if(!hueDepth.isContinuous()){
        printf("NOT CONTINOUS");
    }

    // Msg header
    auto header = std_msgs::msg::Header();
    header.frame_id = "camera";
    header.stamp = this->get_clock()->now();

    // 2D Image publishing
    auto imgMsg = sensor_msgs::msg::Image();
    imgMsg.header = header;
    imgMsg.width = width_;
    imgMsg.height = height_;
    imgMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    imgMsg.step = width_ * sizeof(uchar)*3;
    imgMsg.is_bigendian = true;
    imgMsg.data.assign(hueDepth.data, hueDepth.data + hueDepth.rows*hueDepth.cols*3);
    imgPub_->publish(std::move(imgMsg));
}

void ToFCVNode::pubDepthPtc(float * data){
        // Header
        auto header = std_msgs::msg::Header();
        rclcpp::Time time;
        header.stamp = this->get_clock()->now();
        header.frame_id = "cam_depth";

        // 3D Point clouds message
        auto ptcMsg = sensor_msgs::msg::PointCloud2();
        ptcMsg.header = header;
        ptcMsg.width = width_;
        ptcMsg.height = height_;
        ptcMsg.is_bigendian = true;

        sensor_msgs::PointCloud2Modifier ptcModif(ptcMsg);
        ptcModif.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(ptcMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ptcMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ptcMsg, "z");
        ptcModif.resize(ptcMsg.width * ptcMsg.height);

        //fill data
        for (size_t i = 0; i < ptcMsg.height * ptcMsg.width; i++, ++iter_x, ++iter_y, ++iter_z)
        {
            float z = data[i * 3 + 2];
            if (z <= 0)
            {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();       
                *iter_z = std::numeric_limits<float>::quiet_NaN();                
            }
            else{
                *iter_x = data[i * 3];
                *iter_y = data[i * 3 + 1];
                *iter_z = z;
            }
        }

        pclPub_->publish(ptcMsg);
}

void ToFCVNode::depthCallback(){
    cv::Mat dFrame;
    float *frameData = (float*)malloc(width_*height_*3*sizeof(float));
    while (rclcpp::ok() && cap_->isConnected())
    {
        cap_->getData(frameData);

        // Cam Info
        auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
        infoMsg->header.frame_id = "cam_depth";
        infoMsg->header.stamp = this->get_clock()->now();
        infoPub_->publish(*infoMsg);
        
        // 2D Image
        pubDepthImage(frameData);

        // 3D Image
        pubDepthPtc(frameData);
    }
}
}
