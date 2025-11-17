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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "cis_scm/Params.hpp"
#include "cis_scm/rgbd_driver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    rclcpp::init(argc, argv);
    std::shared_ptr<cis_scm::RGBDNode> rgbd_node =
        std::make_shared<cis_scm::RGBDNode>("rgbd_node", options);

    std::shared_ptr<cis_scm::RGBParamHandler> rgb_param_handler;
    std::shared_ptr<cis_scm::ToFParamHandler> tof_param_handler;
    try {
        rgb_param_handler = std::make_shared<cis_scm::RGBParamHandler>(rgbd_node);
        tof_param_handler = std::make_shared<cis_scm::ToFParamHandler>(rgbd_node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rgbd_node->get_logger(), "Camera control parameters are not active.");
    }
    rgbd_node->start();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(rgbd_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
