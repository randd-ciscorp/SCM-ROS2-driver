#include <rclcpp/rclcpp.hpp>

#include <tof1_driver/rgbd_driver.hpp>

int main(int argc, char* argv[])
{
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    rclcpp::init(argc, argv);
    std::shared_ptr<tof_driver::RGBDNode> rgbd_node = std::make_shared<tof_driver::RGBDNode>(options);
    rgbd_node->start();
    rclcpp::spin(rgbd_node);
    rclcpp::shutdown();
    return 0;
}