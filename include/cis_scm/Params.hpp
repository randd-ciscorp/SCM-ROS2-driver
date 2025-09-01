#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <string_view>
#include <optional>

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
        inline constexpr std::string_view gamma_correction = "isp.gamma correction";

    }

class RGBParamHandler
{
public:
    RGBParamHandler() : rgb_node_(nullptr) {};
    RGBParamHandler(std::shared_ptr<rclcpp::Node> rgb_node_);

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("Camera Parameter Handler");

    std::shared_ptr<rclcpp::Node> rgb_node_;

#ifndef INTERNAL_DRIVER
    std::unique_ptr<CameraCtrlExtern> cam_ctrl_;
#else
    std::unique_ptr<CameraCtrlIntern> cam_ctrl_;
#endif

    template <typename T>
    rcl_interfaces::msg::ParameterDescriptor setParamDescriptor(uint8_t param_type, T min, T max, T step);

    template <typename T>
    void declareParam(std::string_view param_name, T default_val, std::optional<rcl_interfaces::msg::ParameterDescriptor> desc = std::nullopt);

    void setParam(int param_id, rclcpp::Parameter param, rclcpp::ParameterType param_type);

    rcl_interfaces::msg::SetParametersResult setRGBParamCB(const std::vector<rclcpp::Parameter> &parameters);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle;

};
} // namespace cis_scm
#endif
