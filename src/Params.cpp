#include "cis_scm/Params.hpp"

#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cis_scm/Controls.hpp>

namespace cis_scm
{
RGBParamHandler::RGBParamHandler(std::shared_ptr<rclcpp::Node> node) : rgb_node_(node)
{

#ifndef INTERNAL_DEVICE
    cam_ctrl_ = std::make_unique<CameraCtrlExtern>();
    if (!cam_ctrl_->isCtrlOk())
    {
        rclcpp::shutdown();
    }

#else
    cam_ctrl_ = std::make_unique<CameraCtrlIntern>();
#endif

    declareParam("manual_gain", 0.0);
    declareParam("dewarp_bypass", true);

    callback_handle = rgb_node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& param) -> rcl_interfaces::msg::SetParametersResult {
            return this->setRGBParamCB(param);
        }
    );
    RCLCPP_INFO(rgb_node_->get_logger(), "Parameter Handler initialized");
}

template <typename T>
void RGBParamHandler::declareParam(const std::string &param_name, T default_val)
{
    if (!rgb_node_->has_parameter(param_name))
    {
        rgb_node_->declare_parameter(param_name, default_val);
    }

}

void RGBParamHandler::setParam(int cam_param_id, rclcpp::Parameter param, rclcpp::ParameterType param_type)
{
    if (param.get_type() == param_type)
    {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            // No ROS float ParameterType but camera params are in float
            cam_ctrl_->setControlFloat(cam_param_id, (float)param.as_double());
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            cam_ctrl_->setControlInt(cam_param_id, param.as_int());
        }
    }
    else
    {
        RCLCPP_ERROR(logger_, "Parameter value type does not match");
    }
}

rcl_interfaces::msg::SetParametersResult RGBParamHandler::setRGBParamCB(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (const auto &param : parameters)
    {
    RCLCPP_INFO(rgb_node_->get_logger(), "Parameter settttttttting");
        if (param.get_name() == "manual_gain")
        {
            setParam(rgb_set_param::RGB_SET_MANUAL_GAIN, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == "dewarp_bypass")
        {
            setParam(rgb_set_param::RGB_SET_DEWARP_BYPASS, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else
        {
            RCLCPP_ERROR(logger_, "Unknow parameter");
            res.successful = false;
        }
    }
    return res;
}

} // namespace cis_scm

