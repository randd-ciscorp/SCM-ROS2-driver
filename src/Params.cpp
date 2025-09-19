#include "cis_scm/Params.hpp"

#include <stdlib.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cis_scm/Controls.hpp>

using namespace std::chrono_literals;

namespace cis_scm
{

template <typename T>
rcl_interfaces::msg::ParameterDescriptor ParamHandler::setParamDescriptor(uint8_t param_type, T min, T max, T step)
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
        RCLCPP_ERROR(driver_node_->get_logger(), "Wrong parameter type");
        break;
    }

    return desc;
}

template <typename T>
void ParamHandler::declareParam(std::string_view param_name, T default_val, std::optional<rcl_interfaces::msg::ParameterDescriptor> desc)
{
    std::string param_str = std::string(param_name);
    if (!driver_node_->has_parameter(param_str))
    {
        if (desc)
        {
            driver_node_->declare_parameter(param_str, default_val, desc.value());
        }
        else
        {
            driver_node_->declare_parameter(param_str, default_val);
        }
    }

}

void ParamHandler::setParam(int cam_param_id, rclcpp::Parameter param, rclcpp::ParameterType param_type)
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
        if (param.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY)
        {
            std::vector<float> f_vals(param.as_double_array().begin(), param.as_double_array().end());
            cam_ctrl_->setControlFloatArray(cam_param_id, f_vals.data(), param.as_double_array().size());
        }
    }
    else
    {
        RCLCPP_ERROR(logger_, "Parameter value type does not match");
    }
}

RGBParamHandler::RGBParamHandler(std::shared_ptr<rclcpp::Node> node)
{
    driver_node_ = node;

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

    declareParam(IspRosParams::wb_cc_matrix, std::vector<double>{0.,0.,0.,0.,0.,0.,0.,0.,0.});
    declareParam(IspRosParams::wb_offset_r, 0, setParamDescriptor(ParameterType::PARAMETER_INTEGER, -2048, 2047, 1));
    declareParam(IspRosParams::wb_offset_g, 0, setParamDescriptor(ParameterType::PARAMETER_INTEGER, -2048, 2047, 1));
    declareParam(IspRosParams::wb_offset_b, 0, setParamDescriptor(ParameterType::PARAMETER_INTEGER, -2048, 2047, 1));
    declareParam(IspRosParams::wb_gain_r, 1.0, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 3.999, 0.01));
    declareParam(IspRosParams::wb_gain_gr, 1.0, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 3.999, 0.01));
    declareParam(IspRosParams::wb_gain_gb, 1.0, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 3.999, 0.01));
    declareParam(IspRosParams::wb_gain_b, 1.0, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 3.999, 0.01));

    declareParam(IspRosParams::dewarp_bypass, false);

    declareParam(IspRosParams::gamma_correction, true);

    declareParam(IspRosParams::denoising_prefilter, true);

    declareParam(IspRosParams::bls_sub_r, 40, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1023, 1));
    declareParam(IspRosParams::bls_sub_gr, 40, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1023, 1));
    declareParam(IspRosParams::bls_sub_gb, 40, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1023, 1));
    declareParam(IspRosParams::bls_sub_b, 40, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1023, 1));

    declareParam(IspRosParams::lsc_enable, false);
    declareParam(IspRosParams::hflip, false);
    declareParam(IspRosParams::vflip, false);

    declareParam(IspRosParams::cproc_enable, false);
    declareParam(IspRosParams::cproc_color_space, 3, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 1, 4, 1));
    declareParam(IspRosParams::cproc_brightness, -30, setParamDescriptor(ParameterType::PARAMETER_INTEGER, -127, 127, 1));
    declareParam(IspRosParams::cproc_brightness, 0.804688, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 1.99, 0.000001));
    declareParam(IspRosParams::cproc_brightness, 0.328125, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 1.99, 0.000001));
    declareParam(IspRosParams::cproc_brightness, -17, setParamDescriptor(ParameterType::PARAMETER_INTEGER, -90, 89, 1));

    declareParam(IspRosParams::dpcc_enable, true);

    callback_handle = driver_node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& param) -> rcl_interfaces::msg::SetParametersResult {
            return this->setParamCB(param);
        }
    );
    RCLCPP_INFO(driver_node_->get_logger(), "Parameter Handler initialized");
}

ToFParamHandler::ToFParamHandler(std::shared_ptr<rclcpp::Node> node)
{
    driver_node_ = node;

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

    declareParam(TofRosParams::histo_thresh, 17.5, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 100.0, 17.5));
    declareParam(TofRosParams::histo_length, 15, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 1, 100, 15));
    declareParam(TofRosParams::min_reflect, 7.5, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 50.0, 7.5));
    declareParam(TofRosParams::min_confi, 20.0, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 512.0, 20.0));
    declareParam(TofRosParams::kill_flyind_delta, 0.03, setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 1.0, 0.03));
    declareParam(TofRosParams::integr_time, 1000, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1000, 1000));

    callback_handle = driver_node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& param) -> rcl_interfaces::msg::SetParametersResult {
            return this->setParamCB(param);
        }
    );
    RCLCPP_INFO(driver_node_->get_logger(), "Parameter Handler initialized");
}

rcl_interfaces::msg::SetParametersResult RGBParamHandler::setParamCB(const std::vector<rclcpp::Parameter> &parameters)
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
        else if (param.get_name() == IspRosParams::gamma_correction)
        {
            setParam(rgb_set_param::RGB_SET_GAMMA, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        // TODO: add WB CCMatrix
        else if (param.get_name() == IspRosParams::wb_cc_matrix)
        {
            setParam(rgb_set_param::RGB_SET_WHITE_BALANCE_CC_MATRIX, param, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
        }
        else if (param.get_name() == IspRosParams::wb_offset_r)
        {
            setParam(rgb_set_param::RGB_SET_WHITE_BALANCE_OFFSET_R, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::wb_offset_g)
        {
            setParam(rgb_set_param::RGB_SET_WHITE_BALANCE_OFFSET_G, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::wb_offset_b)
        {
            setParam(rgb_set_param::RGB_SET_WHITE_BALANCE_OFFSET_B, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::wb_gain_r)
        {
            setParam(rgb_set_param::RGB_SET_WHITE_BALANCE_GAIN_R, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == IspRosParams::wb_gain_gr)
        {
            setParam(rgb_set_param::RGB_SET_WHITE_BALANCE_GAIN_GR, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == IspRosParams::wb_gain_gb)
        {
            setParam(rgb_set_param::RGB_SET_WHITE_BALANCE_GAIN_GB, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == IspRosParams::wb_gain_b)
        {
            setParam(rgb_set_param::RGB_SET_WHITE_BALANCE_GAIN_B, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == IspRosParams::denoising_prefilter)
        {
            setParam(rgb_set_param::RGB_SET_DENOISING_PREFILTER, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if (param.get_name() == IspRosParams::bls_sub_r)
        {
            setParam(rgb_set_param::RGB_SET_BLACK_LEVEL_SUBTRUCTION_R, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::bls_sub_gr)
        {
            setParam(rgb_set_param::RGB_SET_BLACK_LEVEL_SUBTRUCTION_GR, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::bls_sub_gb)
        {
            setParam(rgb_set_param::RGB_SET_BLACK_LEVEL_SUBTRUCTION_GB, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::bls_sub_b)
        {
            setParam(rgb_set_param::RGB_SET_BLACK_LEVEL_SUBTRUCTION_B, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::lsc_enable)
        {
            setParam(rgb_set_param::RGB_SET_LENS_SHADING_CORRECTION, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if (param.get_name() == IspRosParams::hflip)
        {
            setParam(rgb_set_param::RGB_SET_HFLIP, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if (param.get_name() == IspRosParams::vflip)
        {
            setParam(rgb_set_param::RGB_SET_VFLIP, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if (param.get_name() == IspRosParams::cproc_enable)
        {
            setParam(rgb_set_param::RGB_SET_COLOR_PROCESSING, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }
        else if (param.get_name() == IspRosParams::cproc_color_space)
        {
            setParam(rgb_set_param::RGB_SET_COLOR_SPACE, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::cproc_brightness)
        {
            setParam(rgb_set_param::RGB_SET_BRIGHTNESS, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::cproc_contrast)
        {
            setParam(rgb_set_param::RGB_SET_CONTRAST, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == IspRosParams::cproc_saturation)
        {
            setParam(rgb_set_param::RGB_SET_SATURATION, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == IspRosParams::cproc_hue)
        {
            setParam(rgb_set_param::RGB_SET_HUE, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == IspRosParams::dpcc_enable)
        {
            setParam(rgb_set_param::RGB_SET_DEFECT_PIXEL_CLUSTER_CORRECTION, param, rclcpp::ParameterType::PARAMETER_BOOL);
        }

    }
    return res;
}

rcl_interfaces::msg::SetParametersResult ToFParamHandler::setParamCB(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (const auto &param : parameters)
    {
        if (param.get_name() == TofRosParams::histo_thresh)
        {
            setParam(tof_set_param::TOF_SET_HISTORY_THRESHOLD, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == TofRosParams::histo_length)
        {
            setParam(tof_set_param::TOF_SET_HISTORY_LENGTH, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }
        else if (param.get_name() == TofRosParams::min_reflect)
        {
            setParam(tof_set_param::TOF_SET_MIN_REFLECTANCE, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == TofRosParams::min_confi)
        {
            setParam(tof_set_param::TOF_SET_MIN_CONFIDENCE, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == TofRosParams::kill_flyind_delta)
        {
            setParam(tof_set_param::TOF_SET_KILL_FLYING_DELTA, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        }
        else if (param.get_name() == TofRosParams::integr_time)
        {
            setParam(tof_set_param::TOF_SET_INTEGRATION_TIME, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        }

    }
    return res;
}

} // namespace cis_scm

