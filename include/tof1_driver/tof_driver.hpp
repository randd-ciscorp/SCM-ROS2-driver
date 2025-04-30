#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "tof1_driver/Device.h"

namespace tof_driver
{

class ToFCVNode : public rclcpp::Node
{
public:
    ToFCVNode(const rclcpp::NodeOptions & node_options);

    void start();

private:
    int width_;
    int height_;

    std::vector<std::vector<float>> xyzData_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<Device> cap_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imgPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pclPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoPub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    std::string topicPrefix_ = "camera/depth";
    std::string cameraBaseFrame_ = "camera_base";
    std::string depthCameraFrame_ = "depth_camera_link";

    void importParams();

    void normalize(float* data);
    std::vector<std::vector<float>> splitXYZ(float* data);

    void dispInfo(DevInfo devInfo);

    void pubDepthImage(float * data);
    void pubDepthPtc(float * data);

    void depthCallback();
    void infoCallback();
};
}