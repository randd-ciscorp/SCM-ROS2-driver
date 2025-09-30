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
