#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "cis_scm/rgb_driver.hpp"
#include "cis_scm/Params.hpp"

int main(int argc, char* argv[])
{
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);


    rclcpp::init(argc, argv);
    std::shared_ptr<cis_scm::RGBNode> rgb_node = std::make_shared<cis_scm::RGBNode>("rgb_node", options);
    rgb_node->initImageTransport();
    auto param_handler = std::make_shared<cis_scm::RGBParamHandler>(rgb_node);
    rgb_node->start();
    rclcpp::spin(rgb_node);
    rclcpp::shutdown();

    return 0;
}
