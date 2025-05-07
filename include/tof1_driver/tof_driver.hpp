#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "tof1_driver/Device.h"

#define MAX_DEPTH 7.5

namespace tof_driver
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
    ToFCVNode(const rclcpp::NodeOptions & node_options);

    void start();

private:
    int width_;
    int height_;

    XYZData xyzData_;

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

    XYZData splitXYZ(float* data);

    void dispInfo(DevInfo devInfo) const;

    void pubDepthImage(float * data);
    void pubDepthPtc(float * data);

    void depthCallback();
    void infoCallback();
};
}