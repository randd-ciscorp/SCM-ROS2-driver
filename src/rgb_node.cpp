#include <rclcpp/rclcpp.hpp>

#include "cis_scm/rgb_driver.hpp"

int main(int argc, char* argv[])
{
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    rclcpp::init(argc, argv);
    std::shared_ptr<cis_scm::RGBNode> rgb_node = std::make_shared<cis_scm::RGBNode>("rgb_node", options);
    rgb_node->start();
    rclcpp::spin(rgb_node);
    rclcpp::shutdown();

    return 0;
}
