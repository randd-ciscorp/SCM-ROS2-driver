#ifndef RGBD_DRIVER_HPP
#define RGBD_DRIVER_HPP

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
    #include "cis_scm/Device.h"
#else
    #include "cis_scm/InternalDevice.hpp"
#endif
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

class RGBDNode : public ToFCVNode
{
public:
    RGBDNode(const std::string node_name, const rclcpp::NodeOptions & node_options);

    void start() override;

protected:
    std::vector<uint8_t> frameData_;

    sensor_msgs::msg::Image imgRGBMsg_;
    bool isPCLNoColor_;

#ifdef INTERNAL_DRIVER
    std::unique_ptr<internal::RGBDInternalDevice> cap_;
#endif

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> rgbImgPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoRGBPub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_rgb_;

    std::string rgbCameraFrame_ = "rgb_camera_link";

    int initCap() override;
    void importRGBDParameters();

    XYZRGBData splitXYZRGBData(uint8_t *xyzrgbData);

    void pubRGBImage(uint8_t *data);
    void pubRGBDPtc(XYZRGBData& xyzrgbdata);

    void RGBDCallback();
};
}
#endif
