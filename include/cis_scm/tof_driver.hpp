#ifndef TOF_DRIVER_HPP
#define TOF_DRIVER_HPP

#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <opencv2/opencv.hpp>

#ifndef INTERNAL_DRIVER
    #include "cis_scm/ExternalDevice.hpp"
#else
    #include "cis_scm/InternalDevice.hpp"
#endif

#define MAX_DEPTH 7.5

namespace cis_scm
{

struct XYZData
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
};

class ToFCVNode : public rclcpp::Node
{
public:
    ToFCVNode(const std::string node_name, const rclcpp::NodeOptions & node_options);

    virtual void start();

protected:
    int width_;
    int height_;

    cv::Mat greyDepth_;
    cv::Mat hueDepth_;

    sensor_msgs::msg::PointCloud2 ptcMsg_;
    sensor_msgs::msg::Image imgMsg_;

    XYZData xyzData_;

    rclcpp::TimerBase::SharedPtr timer_;

#ifndef INTERNAL_DRIVER
    std::unique_ptr<ExternalDevice> cap_;
#else
    std::unique_ptr<internal::ToFInternalDevice> cap_;
#endif
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> depthImgPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> depthPCLPub_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoDepthPub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_depth_;

    std::string topicPrefix_ = "camera/depth";
    std::string cameraBaseFrame_ = "camera_base";
    std::string depthCameraFrame_ = "depth_camera_link";

    virtual int initCap();
    void importParams();

    XYZData splitXYZ(float* data);

    void dispInfo(DevInfo devInfo) const;

    void pubDepthImage(float * data);
    void pubDepthPtc(XYZData & data);

    void depthCallback();
    void infoCallback();
};
}
#endif
