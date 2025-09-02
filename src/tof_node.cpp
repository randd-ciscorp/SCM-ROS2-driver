#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "cis_scm/tof_driver.hpp"

int main(int argc, char* argv[]){
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    rclcpp::init(argc, argv);
    std::shared_ptr<cis_scm::ToFCVNode> tof_node = std::make_shared<cis_scm::ToFCVNode>("tof_node", options);
#ifdef INTERNAL_DRIVER
    tof_node->initPointCloudTransport();
#endif
    tof_node->start();
    rclcpp::spin(tof_node);
    rclcpp::shutdown();
    return 0;
}
