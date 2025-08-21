#include "cis_scm/Params.hpp"

#include <stdlib.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cis_scm/Controls.hpp>

using namespace std::chrono_literals;

namespace cis_scm
{
RGBParamHandler::RGBParamHandler(std::shared_ptr<rclcpp::Node> node) : rgb_node_(node)
{

#ifndef INTERNAL_DRIVER
    cam_ctrl_ = std::make_unique<CameraCtrlExtern>();
    if (!cam_ctrl_->isCtrlOk())
    {
        rclcpp::shutdown();
    }

#else
    cam_ctrl_ = std::make_unique<CameraCtrlIntern>();
#endif

    using rcl_interfaces::msg::ParameterType;
    declareParam(IspRosParams::ae_enable, true);
    declareParam(IspRosParams::integration_time, 0.3333, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.000276, 0.3333, 0.000001));
    declareParam(IspRosParams::manual_gain, 1.0, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 16.0, 0.1));

    declareParam(IspRosParams::awb_enable, true);
    declareParam(IspRosParams::awb_index, 0, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 4, 1));

    declareParam(IspRosParams::dewarp_bypass, false);

    declareParam(IspRosParams::gamma_correction, true);

    callback_handle = rgb_node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& param) -> rcl_interfaces::msg::SetParametersResult {
            return this->setRGBParamCB(param);
        }
    );
    RCLCPP_INFO(rgb_node_->get_logger(), "Parameter Handler initialized");
}

template <typename T>
rcl_interfaces::msg::ParameterDescriptor RGBParamHandler::setParamDescriptor(uint8_t param_type, T min, T max, T step)
{
    rcl_interfaces::msg::ParameterDescriptor desc{};
    switch (param_type)
    {
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER: {
        rcl_interfaces::msg::IntegerRange frange{};
        frange.from_value = min;
        frange.to_value = max;
        // step constraints are not enforced by rqt
        //frange.step = step;
        desc.integer_range.push_back(frange);
        break;
    }
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE: {
        rcl_interfaces::msg::FloatingPointRange irange{};
        irange.from_value = min;
        irange.to_value = max;
        // step constraints are not enforced by rqt
        //irange.step = step;
        desc.floating_point_range.push_back(irange);
        break;
    }
    default:
        RCLCPP_ERROR(rgb_node_->get_logger(), "Wrong parameter type");
        break;
    }

    return desc;
}

template <typename T>
void RGBParamHandler::declareParam(std::string_view param_name, T default_val, std::optional<rcl_interfaces::msg::ParameterDescriptor> desc)
{
    std::string param_str = std::string(param_name);
    if (!rgb_node_->has_parameter(param_str))
    {
        if (desc)
        {
            rgb_node_->declare_parameter(param_str, default_val, desc.value());
        }
        else
        {
            rgb_node_->declare_parameter(param_str, default_val);
        }
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
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        {
            cam_ctrl_->setControlInt(cam_param_id, param.as_bool());
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
        if (param.get_name() == IspRosParams::manual_gain)
        {
            setParam(rgb_set_param::RGB_SET_MANUAL_GAIN, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == IspRosParams::ae_enable)
        {
            setParam(rgb_set_param::RGB_SET_AUTO_EXPOSURE_CONTROL, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if (param.get_name() == IspRosParams::integration_time)
        {
            setParam(rgb_set_param::RGB_SET_INTEGRATION_TIME, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == IspRosParams::awb_enable)
        {
            setParam(rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE, param, rclcpp::ParameterType::PARAMETER_BOOL);
            rclcpp::sleep_for(100ms);
            cam_ctrl_->setControlBool(rgb_set_param::RGB_SET_RESET_WHITE_BALANCE_CONTROL, true);
        }
        else if (param.get_name() == IspRosParams::awb_index)
        {
            cam_ctrl_->setControlBool(rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE, false);
            rclcpp::sleep_for(100ms);
            cam_ctrl_->setControlInt(rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE_MODE, 1);

            rclcpp::sleep_for(100ms);
            setParam(rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE_INDEX, param, rclcpp::ParameterType::PARAMETER_INTEGER);
            rclcpp::sleep_for(100ms);
            cam_ctrl_->setControlBool(rgb_set_param::RGB_SET_RESET_WHITE_BALANCE_CONTROL, true);
            rclcpp::sleep_for(100ms);
            cam_ctrl_->setControlBool(rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE, true);

        }
        else if (param.get_name() == IspRosParams::dewarp_bypass)
        {
            setParam(rgb_set_param::RGB_SET_DEWARP_BYPASS, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if (param.get_name() == IspRosParams::gamma_correction)
        {
            setParam(rgb_set_param::RGB_SET_GAMMA, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if (param.get_name() == "camera.rgb.image_raw.enable_pub_plugins")
        {
            // Parameter handled by image_transport
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

