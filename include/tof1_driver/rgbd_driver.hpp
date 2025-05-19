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

#include "tof1_driver/Device.h"
#include "tof1_driver/tof_driver.hpp"

namespace tof_driver
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
    RGBDNode(const rclcpp::NodeOptions & node_options);

    void start();

protected:
    std::vector<uint8_t> frameData_;

    sensor_msgs::msg::Image imgRGBMsg_;
    bool isPCLNoColor_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> rgbImgPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoRGBPub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_rgb_;

    std::string rgbCameraFrame_ = "rgb_camera_link";

    XYZRGBData splitXYZRGBData(uint8_t *xyzrgbData);

    void pubRGBImage(uint8_t *data);
    void pubRGBDPtc(XYZRGBData& xyzrgbdata);

    void RGBDCallback();
};
}
#endif