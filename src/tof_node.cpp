#include <rclcpp/rclcpp.hpp>
#include <tof1_driver/tof_driver.hpp>

int main(int argc, char* argv[]){

rclcpp::NodeOptions options;
options.use_intra_process_comms(true);

rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<tof_driver::ToFCVNode>(options));
rclcpp::shutdown();
return 0;
}