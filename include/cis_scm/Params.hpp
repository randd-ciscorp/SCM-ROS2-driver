// Copyright 2025 CIS Corporation
#ifndef CIS_SCM__PARAMS_HPP_
#define CIS_SCM__PARAMS_HPP_

#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "cis_scm/Controls.hpp"

namespace cis_scm
{

namespace IspRosParams
{
inline constexpr std::string_view ae_enable = "isp.ae.enable";
inline constexpr std::string_view manual_gain = "isp.ae.gain";
inline constexpr std::string_view integration_time = "isp.ae.integration time";
inline constexpr std::string_view awb_enable = "isp.awb.enable";
inline constexpr std::string_view awb_mode = "isp.awb.mode";
inline constexpr std::string_view awb_index = "isp.awb.index";
inline constexpr std::string_view dewarp_bypass = "isp.dewarp bypass";
inline constexpr std::string_view hflip = "isp.Hflip";
inline constexpr std::string_view vflip = "isp.Vflip";
inline constexpr std::string_view gamma_correction = "isp.gamma correction";
inline constexpr std::string_view wb_cc_matrix = "isp.wb.CC matrix";
inline constexpr std::string_view wb_offset_r = "isp.wb.offsetR";
inline constexpr std::string_view wb_offset_g = "isp.wb.offsetG";
inline constexpr std::string_view wb_offset_b = "isp.wb.offsetB";
inline constexpr std::string_view wb_gain_r = "isp.wb.gainR";
inline constexpr std::string_view wb_gain_gr = "isp.wb.gainGR";
inline constexpr std::string_view wb_gain_gb = "isp.wb.gainGB";
inline constexpr std::string_view wb_gain_b = "isp.wb.gainB";
inline constexpr std::string_view denoising_prefilter = "isp.Denoising Prefilter";
inline constexpr std::string_view bls_sub_r = "isp.bls.subtractionR";
inline constexpr std::string_view bls_sub_gr = "isp.bls.subtractionGR";
inline constexpr std::string_view bls_sub_gb = "isp.bls.subtractionGB";
inline constexpr std::string_view bls_sub_b = "isp.bls.subtractionB";
inline constexpr std::string_view lsc_enable = "isp.Lens Shade Correction";
inline constexpr std::string_view cproc_enable = "isp.cproc.Color processing";
inline constexpr std::string_view cproc_color_space = "isp.cproc.Color Space";
inline constexpr std::string_view cproc_brightness = "isp.cproc.brightness";
inline constexpr std::string_view cproc_contrast = "isp.cproc.contrast";
inline constexpr std::string_view cproc_saturation = "isp.cproc.saturation";
inline constexpr std::string_view cproc_hue = "isp.cproc.hue";
inline constexpr std::string_view dpcc_enable = "isp.Defect Pixel Cluster correction";
}  // namespace IspRosParams

namespace TofRosParams
{
inline constexpr std::string_view histo_thresh = "tof.history threshold";
inline constexpr std::string_view histo_length = "tof.history length";
inline constexpr std::string_view min_reflect = "tof.min reflectance";
inline constexpr std::string_view min_confi = "tof.min condidence";
inline constexpr std::string_view kill_flyind_delta = "tof.kill flying delta";
inline constexpr std::string_view integr_time = "tof.integration time";
}  // namespace TofRosParams

class ParamHandler
{
  public:
    virtual ~ParamHandler() = default;

  protected:
    rclcpp::Logger logger_ = rclcpp::get_logger("Camera Parameter Handler");

    std::shared_ptr<rclcpp::Node> driver_node_;

#ifndef INTERNAL_DRIVER
    std::unique_ptr<CameraCtrlExtern> cam_ctrl_;
#else
    std::unique_ptr<CameraCtrlIntern> cam_ctrl_;
#endif

    template <typename T>
    rcl_interfaces::msg::ParameterDescriptor setParamDescriptor(
        uint8_t param_type, T min, T max, T step);

    template <typename T>
    void declareParam(
        std::string_view param_name, T default_val,
        std::optional<rcl_interfaces::msg::ParameterDescriptor> desc = std::nullopt);

    void setParam(int param_id, rclcpp::Parameter param, rclcpp::ParameterType param_type);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle;

    virtual rcl_interfaces::msg::SetParametersResult setParamCB(
        const std::vector<rclcpp::Parameter> & parameters) = 0;
};

class RGBParamHandler : public ParamHandler
{
  public:
    explicit RGBParamHandler(std::shared_ptr<rclcpp::Node> driver_node_);

  private:
    rcl_interfaces::msg::SetParametersResult setParamCB(
        const std::vector<rclcpp::Parameter> & parameters) override;
};

class ToFParamHandler : public ParamHandler
{
  public:
    explicit ToFParamHandler(std::shared_ptr<rclcpp::Node> driver_node_);

  private:
    rcl_interfaces::msg::SetParametersResult setParamCB(
        const std::vector<rclcpp::Parameter> & parameters) override;
};
}  // namespace cis_scm
#endif  // CIS_SCM__PARAMS_HPP_
