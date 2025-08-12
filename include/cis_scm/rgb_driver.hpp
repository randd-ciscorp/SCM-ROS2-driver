#ifndef RGB_DRIVER_HPP
#define RGB_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <opencv2/opencv.hpp>

#ifndef INTERNAL_DRIVER
    #include "cis_scm/ExternalDevice.hpp"
#else
    #include "cis_scm/InternalDevice.hpp"
#endif
#include "cis_scm/Params.hpp"
#include "cis_scm/Controls.hpp"

namespace cis_scm
{

class RGBNode : public rclcpp::Node
{
public:
    RGBNode(const std::string node_name, const rclcpp::NodeOptions & node_options);

    void initParamHandler();
    void start();
    rcl_interfaces::msg::SetParametersResult parameterCB(const std::vector<rclcpp::Parameter> &parameters);
private:
    int width_;
    int height_;

    float inte_time_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle;

    cv::Mat rgbImg_;

    sensor_msgs::msg::Image imgMsg_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<RGBParamHandler> param_handler_;
#ifndef INTERNAL_DRIVER
    std::unique_ptr<ExternalDevice> cap_;
#else
    std::unique_ptr<internal::RGBInternalDevice> cap_;
#endif

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imgPub_;
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

} // namespace cis_scm

#endif
