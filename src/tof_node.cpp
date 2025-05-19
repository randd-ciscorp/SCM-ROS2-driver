#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "tof1_driver/tof_driver.hpp"

int main(int argc, char* argv[]){
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    rclcpp::init(argc, argv);
    std::shared_ptr<tof_driver::ToFCVNode> tof_node = std::make_shared<tof_driver::ToFCVNode>(options);
    tof_node->start();
    rclcpp::spin(tof_node);
    rclcpp::shutdown();
    return 0;
}