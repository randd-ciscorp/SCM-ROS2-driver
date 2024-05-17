#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "tof1_driver/tof_driver.hpp"

using namespace std::chrono_literals;

namespace tof_driver
{

ToFCVNode::ToFCVNode(const rclcpp::NodeOptions & node_options) : Node("tof_driver", node_options){
    rgbImgPub_ = create_publisher<sensor_msgs::msg::Image>(topic_prefix + "/img_rgb", 10);
    depthImgPub_ = create_publisher<sensor_msgs::msg::Image>(topic_prefix + "/img_depth", 10);
    irImgPub_ = create_publisher<sensor_msgs::msg::Image>(topic_prefix + "/img_ir", 10);
    pclPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_prefix + "/pcl_depth", 10);

    infoPub_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic_prefix + "/cam_info", 10);
    infoDepthPub_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic_prefix + "/cam_depth_info", 10);
    infoRGBPub_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic_prefix + "/cam_rgb_info", 10);
    infoIRPub_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic_prefix + "/cam_ir_info", 10);

    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "scm-rgbd");
    cinfo_rgb_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    cinfo_depth_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    cinfo_ir_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    importParams();

    cap_ = std::make_shared<Device>();
    cap_->Connect("0x0001", 480, 640);
    width_ = 640;
    height_ = 480;

    std::cout << "Is connected: " << isConnected() << std::endl; 
    getInfo();
    if (isConnected())
    {
        this->start();
    }
}

bool ToFCVNode::isConnected(){
    return cap_->m_StreamOn;
}

void ToFCVNode::getInfo(){
    printf("info");
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

    // ToF
    //timer_ = this->create_wall_timer(30ms, std::bind(&ToFCVNode::DepthCallback, this));

    // RGBD
    timer_ = this->create_wall_timer(30ms, std::bind(&ToFCVNode::RGBDCallback, this));
}

void ToFCVNode::InfoCallback(){
    while(rclcpp::ok() && isConnected()){
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

void ToFCVNode::pubRGBImage(uint8_t *data){
    auto header = std_msgs::msg::Header();
    header.frame_id = "camera_rgb";
    header.stamp = this->get_clock()->now();

    auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_rgb_->getCameraInfo());
    infoMsg->header = header;
    infoRGBPub_->publish(*infoMsg);

    auto imgMsg = sensor_msgs::msg::Image();
    imgMsg.header = header;
    imgMsg.width = width_;
    imgMsg.height = height_;
    imgMsg.encoding = sensor_msgs::image_encodings::RGB8;
    imgMsg.step = width_ * sizeof(uint8_t) * 3;
    imgMsg.is_bigendian = true;
    imgMsg.data.assign(data, data + imgMsg.height*imgMsg.step);
    rgbImgPub_->publish(std::move(imgMsg));
}

void ToFCVNode::pubIrImage(uint8_t *data){
    auto header = std_msgs::msg::Header();
    header.frame_id = "camera";
    header.stamp = this->get_clock()->now();

    auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_ir_->getCameraInfo());
    infoMsg->header = header;
    infoIRPub_->publish(*infoMsg);

    auto imgMsg = sensor_msgs::msg::Image();
    imgMsg.header = header;
    imgMsg.width = width_;
    imgMsg.height = height_;
    imgMsg.encoding = sensor_msgs::image_encodings::MONO8;
    imgMsg.step = width_;
    imgMsg.is_bigendian = true;
    imgMsg.data.assign(data, data + imgMsg.height * imgMsg.step);
    irImgPub_->publish(imgMsg);
}

void ToFCVNode::pubDepthImage(float* data){
    xyzData_ = splitXYZ(data);
    
    normalize(xyzData_[2].data());
    std::vector<uchar> depth_u8 = std::vector<uchar>(xyzData_[2].size());
    for (size_t i = 0; i < xyzData_[2].size(); i++)
    {
        depth_u8[i] = (uchar)xyzData_[2][i];
    }
    // Msg header
    auto header = std_msgs::msg::Header();
    header.frame_id = "camera_depth";
    header.stamp = this->get_clock()->now();

    auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_depth_->getCameraInfo());
    infoMsg->header = header;
    infoDepthPub_->publish(*infoMsg);

    // 2D Image publishing
    auto imgMsg = sensor_msgs::msg::Image();
    imgMsg.header = header;
    imgMsg.width = width_;
    imgMsg.height = height_;
    imgMsg.encoding = sensor_msgs::image_encodings::MONO8;
    imgMsg.step = width_ * sizeof(uchar);
    imgMsg.is_bigendian = true;
    imgMsg.data.assign(depth_u8.data(), depth_u8.data() + height_*width_);
    depthImgPub_->publish(std::move(imgMsg));
}

void ToFCVNode::pubDepthPtc(float *xyzData, uint8_t *rgbData){
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

    if (rgbData)
    {
        sensor_msgs::PointCloud2Modifier ptcModif(ptcMsg);
        ptcModif.setPointCloud2FieldsByString(2, "xyz", "rgb");

        sensor_msgs::PointCloud2Iterator<float> iter_x(ptcMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ptcMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ptcMsg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(ptcMsg, "rgb");
        ptcModif.resize(ptcMsg.width * ptcMsg.height);

        //fill data
        for (size_t i = 0; i < ptcMsg.height * ptcMsg.width; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
        {
            float z = xyzData[i * 3 + 2];
            if (z <= 0)
            {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();       
                *iter_z = std::numeric_limits<float>::quiet_NaN();      
                *iter_rgb = 0;
            }
            else{
                *iter_x = xyzData[i * 3];
                *iter_y = xyzData[i * 3 + 1];
                *iter_z = z;
                iter_rgb[0] = rgbData[i * 3+2];
                iter_rgb[1] = rgbData[i * 3+1];
                iter_rgb[2] = rgbData[i * 3+0];
            }
        }
    }
    else
    {
        sensor_msgs::PointCloud2Modifier ptcModif(ptcMsg);
        ptcModif.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(ptcMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ptcMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ptcMsg, "z");
        ptcModif.resize(ptcMsg.width * ptcMsg.height);

        //fill data
        for (size_t i = 0; i < ptcMsg.height * ptcMsg.width; i++, ++iter_x, ++iter_y, ++iter_z)
        {
            float z = xyzData[i * 3 + 2];
            if (z <= 0)
            {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();       
                *iter_z = std::numeric_limits<float>::quiet_NaN();                
            }
            else{
                *iter_x = xyzData[i * 3];
                *iter_y = xyzData[i * 3 + 1];
                *iter_z = z;
            }
        }
    }
    
    
    pclPub_->publish(ptcMsg);
}

void ToFCVNode::RGBDCallback(){
    
    int nbBytePerPxl = sizeof(float)*3 + sizeof(uint8_t)*3 + sizeof(uint8_t);
    
    while (rclcpp::ok() && isConnected())
    {
        uint8_t *rgb_ptr = new uint8_t[width_ * height_ * 3];
        float *xyz_ptr = new float[width_ * height_ * 3];
        uint8_t *ir_ptr = new uint8_t[width_ * height_];

        pclData *structFrameData = new pclData[width_ * height_];
        cap_->GetData(structFrameData);

        // Cam Info
        auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
        infoMsg->header.frame_id = "camera";
        infoMsg->header.stamp = this->get_clock()->now();
        infoPub_->publish(*infoMsg);
        
        // Split XYZ RGB IR
        for (id_t i = 0; i < width_ * height_; i++)
        {
            xyz_ptr[3*i] = structFrameData[i].x;
            xyz_ptr[3*i+1] = structFrameData[i].y;
            xyz_ptr[3*i+2] = structFrameData[i].z;
            rgb_ptr[3*i] = structFrameData[i].r;
            rgb_ptr[3*i+1] = structFrameData[i].g;
            rgb_ptr[3*i+2] = structFrameData[i].b;
            ir_ptr[3*i+2] = structFrameData[i].a;
        }

        pubRGBImage(rgb_ptr);
        //pubIrImage(ir_ptr);
        pubDepthImage(xyz_ptr);
        pubDepthPtc(xyz_ptr, rgb_ptr);
    }
}
}
