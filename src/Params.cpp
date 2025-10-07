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

#include "cis_scm/Params.hpp"

#include <stdlib.h>

#include <cassert>
#include <chrono>
#include <cis_scm/Controls.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

namespace cis_scm
{

template <typename T>
rcl_interfaces::msg::ParameterDescriptor ParamHandler::setParamDescriptor(
    uint8_t param_type, T min, T max, T step)
{
    rcl_interfaces::msg::ParameterDescriptor desc{};
    switch (param_type) {
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER: {
            rcl_interfaces::msg::IntegerRange frange{};
            frange.from_value = min;
            frange.to_value = max;
            // step constraints are not enforced by rqt
            // frange.step = step;
            desc.integer_range.push_back(frange);
            break;
        }
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE: {
            rcl_interfaces::msg::FloatingPointRange irange{};
            irange.from_value = min;
            irange.to_value = max;
            // step constraints are not enforced by rqt
            // irange.step = step;
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
void ParamHandler::declareParam(
    std::string_view param_name, T default_val,
    std::optional<rcl_interfaces::msg::ParameterDescriptor> desc)
{
    std::string param_str = std::string(param_name);
    if (!driver_node_->has_parameter(param_str)) {
        if (desc) {
            driver_node_->declare_parameter(param_str, default_val, desc.value());
        } else {
            driver_node_->declare_parameter(param_str, default_val);
        }
    }
}

void ParamHandler::setParam(
    int cam_param_id, rclcpp::Parameter param, rclcpp::ParameterType param_type)
{
    if (param.get_type() == param_type) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            // No ROS float ParameterType but camera params are in float
            cam_ctrl_->setControlFloat(cam_param_id, static_cast<float>(param.as_double()));
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            cam_ctrl_->setControlInt(cam_param_id, param.as_int());
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            cam_ctrl_->setControlInt(cam_param_id, param.as_bool());
        }
        if (param.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY) {
            std::vector<float> f_vals(
                param.as_double_array().begin(), param.as_double_array().end());
            cam_ctrl_->setControlFloatArray(
                cam_param_id, f_vals.data(), param.as_double_array().size());
        }
    } else {
        RCLCPP_ERROR(logger_, "Parameter value type does not match");
    }
}

template <typename T>
rclcpp::Parameter ParamHandler::makeParamFromCtrlVal(std::string_view param_name, int ctrl_id)
{
    // Avoid triggering the callback add_on_set_parameters_callback, this avoid get-set-get-set infinite behavior
    ignore_set_param_cb_ = true;
    std::string param_name_str = std::string(param_name);
    T param_val;
    int ret_get = -1;
    if constexpr (std::is_same_v<T, int>) {
        ret_get = cam_ctrl_->getControlInt(ctrl_id, param_val);
    } else if constexpr (std::is_same_v<T, float>) {
        ret_get = cam_ctrl_->getControlFloat(ctrl_id, param_val);
    } else if constexpr (std::is_same_v<T, bool>) {
        ret_get = cam_ctrl_->getControlBool(ctrl_id, param_val);
    } else if constexpr (std::is_array_v<T> && std::is_same_v<std::remove_extent_t<T>, float>) {
        constexpr int array_size = std::extent_v<T>;
        std::vector<float> vec_param_vals;
        ret_get = cam_ctrl_->getControlFloatArray(ctrl_id, vec_param_vals, array_size);
        return rclcpp::Parameter(param_name_str, vec_param_vals);
    } else {
        std::cout << typeid(T).name() << std::endl;
        // static_assert(false, "Unsupported parameter value type");
        return driver_node_->get_parameter(param_name_str);
    }
    if (!ret_get) {
        return rclcpp::Parameter(param_name_str, param_val);
    } else {
        // If problems with getting ctrl value, just return the value from the last update
        return driver_node_->get_parameter(param_name_str);
    }
}

RGBParamHandler::RGBParamHandler(std::shared_ptr<rclcpp::Node> node)
{
    driver_node_ = node;

#ifndef INTERNAL_DRIVER
    cam_ctrl_ = std::make_unique<CameraCtrlExtern>();
#else
    cam_ctrl_ = std::make_unique<CameraCtrlIntern>();
#endif

    using rcl_interfaces::msg::ParameterType;

    declareParam(IspRosParams::ae_enable, true);
    declareParam(
        IspRosParams::integration_time, 0.3333,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.000276, 0.3333, 0.000001));
    declareParam(
        IspRosParams::manual_gain, 1.0,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 16.0, 0.1));

    declareParam(IspRosParams::awb_enable, true);
    declareParam(
        IspRosParams::awb_index, 0, setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 4, 1));

    declareParam(
        IspRosParams::wb_cc_matrix, std::vector<double>{0., 0., 0., 0., 0., 0., 0., 0., 0.});
    declareParam(
        IspRosParams::wb_offset_r, 0,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, -2048, 2047, 1));
    declareParam(
        IspRosParams::wb_offset_g, 0,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, -2048, 2047, 1));
    declareParam(
        IspRosParams::wb_offset_b, 0,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, -2048, 2047, 1));
    declareParam(
        IspRosParams::wb_gain_r, 1.0,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 3.999, 0.01));
    declareParam(
        IspRosParams::wb_gain_gr, 1.0,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 3.999, 0.01));
    declareParam(
        IspRosParams::wb_gain_gb, 1.0,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 3.999, 0.01));
    declareParam(
        IspRosParams::wb_gain_b, 1.0,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 1.0, 3.999, 0.01));

    declareParam(IspRosParams::dewarp_bypass, false);

    declareParam(IspRosParams::gamma_correction, true);

    declareParam(IspRosParams::denoising_prefilter, true);

    declareParam(
        IspRosParams::bls_sub_r, 40,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1023, 1));
    declareParam(
        IspRosParams::bls_sub_gr, 40,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1023, 1));
    declareParam(
        IspRosParams::bls_sub_gb, 40,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1023, 1));
    declareParam(
        IspRosParams::bls_sub_b, 40,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1023, 1));

    declareParam(IspRosParams::lsc_enable, false);
    declareParam(IspRosParams::hflip, false);
    declareParam(IspRosParams::vflip, false);

    declareParam(IspRosParams::cproc_enable, false);
    declareParam(
        IspRosParams::cproc_color_space, 3,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, 1, 4, 1));
    declareParam(
        IspRosParams::cproc_brightness, -30,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, -127, 127, 1));
    declareParam(
        IspRosParams::cproc_contrast, 0.804688,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 1.99, 0.000001));
    declareParam(
        IspRosParams::cproc_saturation, 1.328125,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 1.99, 0.000001));
    declareParam(
        IspRosParams::cproc_hue, -17,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, -90, 89, 1));

    declareParam(IspRosParams::dpcc_enable, true);

    params_cb_grp_ =
        driver_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_handle = driver_node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & param)
            -> rcl_interfaces::msg::SetParametersResult {
            if (ignore_set_param_cb_) {
                ignore_set_param_cb_ = false;
                return rcl_interfaces::msg::SetParametersResult().set__successful(true);
            }
            return this->setParamCB(param);
        });

    timer_ = driver_node_->create_wall_timer(
        std::chrono::milliseconds(1000), [this]() { updateControlsParams(); }, params_cb_grp_);

    RCLCPP_INFO(driver_node_->get_logger(), "Parameter Handler initialized");
}

ToFParamHandler::ToFParamHandler(std::shared_ptr<rclcpp::Node> node)
{
    driver_node_ = node;

#ifndef INTERNAL_DRIVER
    cam_ctrl_ = std::make_unique<CameraCtrlExtern>();
#else
    cam_ctrl_ = std::make_unique<CameraCtrlIntern>();
#endif

    using rcl_interfaces::msg::ParameterType;

    declareParam(
        TofRosParams::histo_thresh, 17.5,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 100.0, 17.5));
    declareParam(
        TofRosParams::histo_length, 15,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, 1, 100, 15));
    declareParam(
        TofRosParams::min_reflect, 7.5,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 50.0, 7.5));
    declareParam(
        TofRosParams::min_confi, 20.0,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 512.0, 20.0));
    declareParam(
        TofRosParams::kill_flyind_delta, 0.03,
        setParamDescriptor(ParameterType::PARAMETER_DOUBLE, 0.0, 1.0, 0.03));
    declareParam(
        TofRosParams::integr_time, 1000,
        setParamDescriptor(ParameterType::PARAMETER_INTEGER, 0, 1000, 1000));

    params_cb_grp_ =
        driver_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_handle = driver_node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & param)
            -> rcl_interfaces::msg::SetParametersResult {
            if (ignore_set_param_cb_) {
                ignore_set_param_cb_ = false;
                return rcl_interfaces::msg::SetParametersResult().set__successful(true);
            }
            return this->setParamCB(param);
        });

    timer_ = driver_node_->create_wall_timer(
        std::chrono::milliseconds(1000), [this]() { updateControlsParams(); }, params_cb_grp_);

    RCLCPP_INFO(driver_node_->get_logger(), "Parameter Handler initialized");
}

rcl_interfaces::msg::SetParametersResult RGBParamHandler::setParamCB(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (const auto & param : parameters) {
        if (param.get_name() == IspRosParams::manual_gain) {
            setParam(
                rgb_set_param::RGB_SET_MANUAL_GAIN, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == IspRosParams::ae_enable) {
            setParam(
                rgb_set_param::RGB_SET_AUTO_EXPOSURE_CONTROL, param,
                rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::integration_time) {
            setParam(
                rgb_set_param::RGB_SET_INTEGRATION_TIME, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == IspRosParams::awb_enable) {
            setParam(
                rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE, param,
                rclcpp::ParameterType::PARAMETER_BOOL);
            rclcpp::sleep_for(100ms);
            cam_ctrl_->setControlBool(rgb_set_param::RGB_SET_RESET_WHITE_BALANCE_CONTROL, true);
        } else if (param.get_name() == IspRosParams::awb_index) {
            cam_ctrl_->setControlBool(rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE, false);
            rclcpp::sleep_for(100ms);
            cam_ctrl_->setControlInt(rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE_MODE, 1);

            rclcpp::sleep_for(100ms);
            setParam(
                rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE_INDEX, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
            rclcpp::sleep_for(100ms);
            cam_ctrl_->setControlBool(rgb_set_param::RGB_SET_RESET_WHITE_BALANCE_CONTROL, true);
            rclcpp::sleep_for(100ms);
            cam_ctrl_->setControlBool(rgb_set_param::RGB_SET_AUTO_WHITE_BALANCE, true);

        } else if (param.get_name() == IspRosParams::dewarp_bypass) {
            setParam(
                rgb_set_param::RGB_SET_DEWARP_BYPASS, param, rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::gamma_correction) {
            setParam(rgb_set_param::RGB_SET_GAMMA, param, rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::gamma_correction) {
            setParam(rgb_set_param::RGB_SET_GAMMA, param, rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::wb_cc_matrix) {
            setParam(
                rgb_set_param::RGB_SET_WHITE_BALANCE_CC_MATRIX, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
        } else if (param.get_name() == IspRosParams::wb_offset_r) {
            setParam(
                rgb_set_param::RGB_SET_WHITE_BALANCE_OFFSET_R, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::wb_offset_g) {
            setParam(
                rgb_set_param::RGB_SET_WHITE_BALANCE_OFFSET_G, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::wb_offset_b) {
            setParam(
                rgb_set_param::RGB_SET_WHITE_BALANCE_OFFSET_B, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::wb_gain_r) {
            setParam(
                rgb_set_param::RGB_SET_WHITE_BALANCE_GAIN_R, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == IspRosParams::wb_gain_gr) {
            setParam(
                rgb_set_param::RGB_SET_WHITE_BALANCE_GAIN_GR, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == IspRosParams::wb_gain_gb) {
            setParam(
                rgb_set_param::RGB_SET_WHITE_BALANCE_GAIN_GB, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == IspRosParams::wb_gain_b) {
            setParam(
                rgb_set_param::RGB_SET_WHITE_BALANCE_GAIN_B, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == IspRosParams::denoising_prefilter) {
            setParam(
                rgb_set_param::RGB_SET_DENOISING_PREFILTER, param,
                rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::bls_sub_r) {
            setParam(
                rgb_set_param::RGB_SET_BLACK_LEVEL_SUBTRUCTION_R, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::bls_sub_gr) {
            setParam(
                rgb_set_param::RGB_SET_BLACK_LEVEL_SUBTRUCTION_GR, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::bls_sub_gb) {
            setParam(
                rgb_set_param::RGB_SET_BLACK_LEVEL_SUBTRUCTION_GB, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::bls_sub_b) {
            setParam(
                rgb_set_param::RGB_SET_BLACK_LEVEL_SUBTRUCTION_B, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::lsc_enable) {
            setParam(
                rgb_set_param::RGB_SET_LENS_SHADING_CORRECTION, param,
                rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::hflip) {
            setParam(rgb_set_param::RGB_SET_HFLIP, param, rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::vflip) {
            setParam(rgb_set_param::RGB_SET_VFLIP, param, rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::cproc_enable) {
            setParam(
                rgb_set_param::RGB_SET_COLOR_PROCESSING, param,
                rclcpp::ParameterType::PARAMETER_BOOL);
        } else if (param.get_name() == IspRosParams::cproc_color_space) {
            setParam(
                rgb_set_param::RGB_SET_COLOR_SPACE, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::cproc_brightness) {
            setParam(
                rgb_set_param::RGB_SET_BRIGHTNESS, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::cproc_contrast) {
            setParam(
                rgb_set_param::RGB_SET_CONTRAST, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == IspRosParams::cproc_saturation) {
            setParam(
                rgb_set_param::RGB_SET_SATURATION, param, rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == IspRosParams::cproc_hue) {
            setParam(rgb_set_param::RGB_SET_HUE, param, rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == IspRosParams::dpcc_enable) {
            setParam(
                rgb_set_param::RGB_SET_DEFECT_PIXEL_CLUSTER_CORRECTION, param,
                rclcpp::ParameterType::PARAMETER_BOOL);
        }
    }
    return res;
}

void RGBParamHandler::updateControlsParams()
{
    driver_node_->set_parameter(
        makeParamFromCtrlVal<bool>(IspRosParams::ae_enable, RGB_GET_AUTO_EXPOSURE_CONTROL));
    // TODO: add AE controls
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(IspRosParams::manual_gain, RGB_GET_MANUAL_GAIN));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(IspRosParams::integration_time, RGB_GET_INTEGRATION_TIME));

    driver_node_->set_parameter(
        makeParamFromCtrlVal<bool>(IspRosParams::awb_enable, RGB_GET_AUTO_WHITE_BALANCE));
    // AWB mode removed --> made a more convenient way with awb_enable and awb_index
    // driver_node_->set_parameter(
    //     makeParamFromCtrlVal<int>(IspRosParams::awb_mode, RGB_GET_AUTO_WHITE_BALANCE_MODE));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::awb_index, RGB_GET_AUTO_WHITE_BALANCE_INDEX));

    // TODO: Add AWB damping?
    //driver_node_->set_parameter(makeParamFromCtrlVal<int>(IspRosPa
    driver_node_->set_parameter(makeParamFromCtrlVal<float[cc_matrix_nb_elems]>(
        IspRosParams::wb_cc_matrix, RGB_GET_WHITE_BALANCE_CC_MATRIX));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::wb_offset_r, RGB_GET_WHITE_BALANCE_OFFGET_R));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::wb_offset_g, RGB_GET_WHITE_BALANCE_OFFSET_G));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::wb_offset_b, RGB_GET_WHITE_BALANCE_OFFSET_B));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(IspRosParams::wb_gain_r, RGB_GET_WHITE_BALANCE_GAIN_R));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(IspRosParams::wb_gain_gr, RGB_GET_WHITE_BALANCE_GAIN_GR));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(IspRosParams::wb_gain_gb, RGB_GET_WHITE_BALANCE_GAIN_GB));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(IspRosParams::wb_gain_b, RGB_GET_WHITE_BALANCE_GAIN_B));

    driver_node_->set_parameter(
        makeParamFromCtrlVal<bool>(IspRosParams::denoising_prefilter, RGB_GET_DENOISING_PREFILTER));

    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::bls_sub_r, RGB_GET_BLACK_LEVEL_SUBTRUCTION_R));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::bls_sub_gr, RGB_GET_BLACK_LEVEL_SUBTRUCTION_GR));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::bls_sub_gb, RGB_GET_BLACK_LEVEL_SUBTRUCTION_GB));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::bls_sub_b, RGB_GET_BLACK_LEVEL_SUBTRUCTION_B));

    driver_node_->set_parameter(
        makeParamFromCtrlVal<bool>(IspRosParams::lsc_enable, RGB_GET_LENS_SHADING_CORRECTION));

    driver_node_->set_parameter(
        makeParamFromCtrlVal<bool>(IspRosParams::dewarp_bypass, RGB_GET_DEWARP_BYPASS));
    driver_node_->set_parameter(makeParamFromCtrlVal<bool>(IspRosParams::hflip, RGB_GET_HFLIP));
    driver_node_->set_parameter(makeParamFromCtrlVal<bool>(IspRosParams::vflip, RGB_GET_VFLIP));

    driver_node_->set_parameter(
        makeParamFromCtrlVal<bool>(IspRosParams::gamma_correction, RGB_GET_GAMMA));

    driver_node_->set_parameter(
        makeParamFromCtrlVal<bool>(IspRosParams::cproc_enable, RGB_GET_COLOR_PROCESSING));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::cproc_color_space, RGB_GET_COLOR_SPACE));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(IspRosParams::cproc_brightness, RGB_GET_BRIGHTNESS));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(IspRosParams::cproc_contrast, RGB_GET_CONTRAST));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(IspRosParams::cproc_saturation, RGB_GET_SATURATION));
    driver_node_->set_parameter(makeParamFromCtrlVal<int>(IspRosParams::cproc_hue, RGB_GET_HUE));

    driver_node_->set_parameter(makeParamFromCtrlVal<int>(
        IspRosParams::dpcc_enable, RGB_GET_DEFECT_PIXEL_CLUSTER_CORRECTION));
}

rcl_interfaces::msg::SetParametersResult ToFParamHandler::setParamCB(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (const auto & param : parameters) {
        if (param.get_name() == TofRosParams::histo_thresh) {
            setParam(
                tof_set_param::TOF_SET_HISTORY_THRESHOLD, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == TofRosParams::histo_length) {
            setParam(
                tof_set_param::TOF_SET_HISTORY_LENGTH, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        } else if (param.get_name() == TofRosParams::min_reflect) {
            setParam(
                tof_set_param::TOF_SET_MIN_REFLECTANCE, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == TofRosParams::min_confi) {
            setParam(
                tof_set_param::TOF_SET_MIN_CONFIDENCE, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == TofRosParams::kill_flyind_delta) {
            setParam(
                tof_set_param::TOF_SET_KILL_FLYING_DELTA, param,
                rclcpp::ParameterType::PARAMETER_DOUBLE);
        } else if (param.get_name() == TofRosParams::integr_time) {
            setParam(
                tof_set_param::TOF_SET_INTEGRATION_TIME, param,
                rclcpp::ParameterType::PARAMETER_INTEGER);
        }
    }
    return res;
}

void ToFParamHandler::updateControlsParams()
{
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(TofRosParams::min_reflect, TOF_GET_MIN_REFLECTANCE));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(TofRosParams::min_confi, TOF_GET_MIN_CONFIDENCE));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<float>(TofRosParams::kill_flyind_delta, TOF_GET_KILL_FLYING_DELTA));
    driver_node_->set_parameter(
        makeParamFromCtrlVal<int>(TofRosParams::integr_time, TOF_GET_INTEGRATION_TIME));
}

}  // namespace cis_scm
