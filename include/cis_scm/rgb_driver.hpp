// Copyright 2025 CIS Corporation
#ifndef CIS_SCM__RGB_DRIVER_HPP_
#define CIS_SCM__RGB_DRIVER_HPP_

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#ifndef INTERNAL_DRIVER
#include "cis_scm/ExternalDevice.hpp"
#else
#include <image_transport/image_transport.hpp>

#include "cis_scm/InternalDevice.hpp"
#endif

namespace cis_scm
{

class RGBNode : public rclcpp::Node
{
  public:
    RGBNode(const std::string node_name, const rclcpp::NodeOptions & get_node_options);

#ifdef INTERNAL_DRIVER
    void initImageTransport();
#endif

    void start();

  private:
    int width_;
    int height_;

    float inte_time_;

    cv::Mat rgbImg_;

    sensor_msgs::msg::Image imgMsg_;

    rclcpp::TimerBase::SharedPtr timer_;

#ifdef INTERNAL_DRIVER
    std::unique_ptr<internal::RGBInternalDevice> cap_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher imgPub_;
#else
    std::unique_ptr<ExternalDevice> cap_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imgPub_;
#endif

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoPub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    std::string topicPrefix_ = "camera/rgb";
    std::string cameraBaseFrame_ = "camera_base";
    std::string rgbCameraFrame_ = "rgb_camera_link";

    int initCap();
    void importParams();
    void dispInfo(DevInfo devInfo) const;

    void pubImage(uint8_t * data);
    void imgCallback();
};

}  // namespace cis_scm

#endif  // CIS_SCM__RGB_DRIVER_HPP_
