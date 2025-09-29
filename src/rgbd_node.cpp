// Copyright 2025 CIS Corporation
#include <cis_scm/rgbd_driver.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    rclcpp::init(argc, argv);
    std::shared_ptr<cis_scm::RGBDNode> rgbd_node =
        std::make_shared<cis_scm::RGBDNode>("rgbd_node", options);
    rgbd_node->start();
    rclcpp::spin(rgbd_node);
    rclcpp::shutdown();
    return 0;
}
