#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "tof1_driver/Device.h"

namespace tof_driver
{

class ToFCVNode : public rclcpp::Node
{
public:
    ToFCVNode(const rclcpp::NodeOptions & node_options);

    void start();

    std::shared_ptr<Device> cap_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imgPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pclPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoPub_;

    rclcpp::TimerBase::SharedPtr timer_;

    int width_;
    int height_;
private:
    bool isConnected();
    std::vector<std::vector<float>> xyzData_;
    std::vector<std::vector<float>> splitXYZ(float* data);

    void normalize(float* data);

    void getFrame();
    void getInfo();
    void pubDepthImage(float * data);
    void pubDepthPtc(float * data);

    void DepthCallback();
    void InfoCallback();
};
}