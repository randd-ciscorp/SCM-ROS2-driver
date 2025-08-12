#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cis_scm/Controls.hpp>

namespace cis_scm
{

class RGBParamHandler
{
public:
    RGBParamHandler() : rgb_node_(nullptr) {};
    RGBParamHandler(std::shared_ptr<rclcpp::Node> rgb_node_);

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("Camera Parameter Handler");

    std::shared_ptr<rclcpp::Node> rgb_node_;

#ifndef INTERNAL_DEVICE
    std::unique_ptr<CameraCtrlExtern> cam_ctrl_;
#else
    std::unique_ptr<CameraCtrlIntern> cam_ctrl_;
#endif


    template <typename T>
    void declareParam(const std::string &param_name, T default_val);
    void setParam(int param_id, rclcpp::Parameter param, rclcpp::ParameterType param_type);

    rcl_interfaces::msg::SetParametersResult setRGBParamCB(const std::vector<rclcpp::Parameter> &parameters);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle;

};
} // namespace cis_scm
#endif
