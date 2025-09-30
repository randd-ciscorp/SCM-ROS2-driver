// Copyright 2025 CIS Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cis_scm/Params.hpp"
#include "cis_scm/rgb_driver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::init(argc, argv);
    std::shared_ptr<cis_scm::RGBNode> rgb_node =
        std::make_shared<cis_scm::RGBNode>("rgb_node", options);
#ifdef INTERNAL_DRIVER
    rgb_node->initImageTransport();
#endif
    try {
        auto param_handler = std::make_shared<cis_scm::RGBParamHandler>(rgb_node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rgb_node->get_logger(), "Camera control parameters are not active.");
    }
    rgb_node->start();
    rclcpp::spin(rgb_node);
    rclcpp::shutdown();

    return 0;
}
