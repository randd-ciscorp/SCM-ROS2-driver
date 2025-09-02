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

    namespace TofRosParams
    {
        inline constexpr std::string_view histo_thresh = "tof.history threshold";
        inline constexpr std::string_view histo_length = "tof.history length";
        inline constexpr std::string_view min_reflect = "tof.min reflectance";
        inline constexpr std::string_view min_confi = "tof.min condidence";
        inline constexpr std::string_view kill_flyind_delta = "tof.kill flying delta";
        inline constexpr std::string_view integr_time = "tof.integration time";
    }

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
    rcl_interfaces::msg::ParameterDescriptor setParamDescriptor(uint8_t param_type, T min, T max, T step);

    template <typename T>
    void declareParam(std::string_view param_name, T default_val, std::optional<rcl_interfaces::msg::ParameterDescriptor> desc = std::nullopt);

    void setParam(int param_id, rclcpp::Parameter param, rclcpp::ParameterType param_type);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle;

    virtual rcl_interfaces::msg::SetParametersResult setParamCB(const std::vector<rclcpp::Parameter> &parameters) = 0;
};

class RGBParamHandler : public ParamHandler
{
public:
    RGBParamHandler(std::shared_ptr<rclcpp::Node> driver_node_);

private:
    rcl_interfaces::msg::SetParametersResult setParamCB(const std::vector<rclcpp::Parameter> &parameters) override;

};

class ToFParamHandler : public ParamHandler
{
public:
    ToFParamHandler(std::shared_ptr<rclcpp::Node> driver_node_);

private:
    rcl_interfaces::msg::SetParametersResult setParamCB(const std::vector<rclcpp::Parameter> &parameters) override;

};
} // namespace cis_scm
#endif
