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

#ifndef CIS_SCM__CONTROLS_HPP_
#define CIS_SCM__CONTROLS_HPP_

#include <sys/types.h>
#include <cstdint>
#include <vector>

#include <rclcpp/parameter_client.hpp>

namespace cis_scm
{

inline constexpr int cc_matrix_nb_elems = 9;

enum tof_set_param
{
    TOF_SET_HISTORY_THRESHOLD = 100,
    TOF_SET_HISTORY_LENGTH = 101,
    TOF_SET_MIN_REFLECTANCE = 102,
    TOF_SET_MIN_CONFIDENCE = 103,
    TOF_SET_KILL_FLYING_DELTA = 104,
    TOF_SET_INTEGRATION_TIME = 105
};

enum tof_get_param
{
    TOF_GET_HISTORY_THRESHOLD = 100,
    TOF_GET_HISTORY_LENGTH = 101,
    TOF_GET_MIN_REFLECTANCE = 102,
    TOF_GET_MIN_CONFIDENCE = 103,
    TOF_GET_KILL_FLYING_DELTA = 104,
    TOF_GET_INTEGRATION_TIME = 105
};

enum rgb_set_param
{
    // AEC
    RGB_SET_AUTO_EXPOSURE_CONTROL = 1,
    RGB_SET_RESET_AUTO_EXPOSURE_CONTROL = 2,
    RGB_SET_AUTO_EXPOSURE_SET_POINT = 3,
    RGB_SET_AUTO_EXPOSURE_CONTROL_DAMP_OVER = 4,
    RGB_SET_AUTO_EXPOSURE_CONTROL_DAMP_UNDER = 5,
    RGB_SET_AUTO_EXPOSURE_CONTROL_TOLERANCE = 6,
    RGB_SET_MANUAL_GAIN = 7,
    RGB_SET_INTEGRATION_TIME = 8,
    RGB_SET_AUTO_EXPOSURE_WEIGHT = 9,

    // AWB
    RGB_SET_AUTO_WHITE_BALANCE = 10,
    RGB_SET_RESET_WHITE_BALANCE_CONTROL = 11,
    RGB_SET_AUTO_WHITE_BALANCE_MODE = 12,
    RGB_SET_AUTO_WHITE_BALANCE_INDEX = 13,
    RGB_SET_AUTO_WHITE_BALANCE_DAMPING = 14,

    // WB
    RGB_SET_WHITE_BALANCE_CC_MATRIX = 15,
    RGB_SET_WHITE_BALANCE_OFFSET_R = 16,
    RGB_SET_WHITE_BALANCE_OFFSET_G = 17,
    RGB_SET_WHITE_BALANCE_OFFSET_B = 18,
    RGB_SET_WHITE_BALANCE_GAIN_R = 19,
    RGB_SET_WHITE_BALANCE_GAIN_GR = 20,
    RGB_SET_WHITE_BALANCE_GAIN_GB = 21,
    RGB_SET_WHITE_BALANCE_GAIN_B = 22,

    // DPF
    RGB_SET_DENOISING_PREFILTER = 23,

    // BLS
    RGB_SET_BLACK_LEVEL_SUBTRUCTION_R = 24,
    RGB_SET_BLACK_LEVEL_SUBTRUCTION_GR = 25,
    RGB_SET_BLACK_LEVEL_SUBTRUCTION_GB = 26,
    RGB_SET_BLACK_LEVEL_SUBTRUCTION_B = 27,

    // LSC
    RGB_SET_LENS_SHADING_CORRECTION = 28,

    // DWE
    RGB_SET_DEWARP_BYPASS = 29,
    RGB_SET_HFLIP = 30,
    RGB_SET_VFLIP = 31,

    // GAMMA
    RGB_SET_GAMMA = 32,

    // CPROC
    RGB_SET_COLOR_PROCESSING = 33,
    RGB_SET_COLOR_SPACE = 34,
    RGB_SET_BRIGHTNESS = 35,
    RGB_SET_CONTRAST = 36,
    RGB_SET_SATURATION = 37,
    RGB_SET_HUE = 38,

    // DPC
    RGB_SET_DEFECT_PIXEL_CLUSTER_CORRECTION = 39
};

enum rgb_get_param
{
    // AEC
    RGB_GET_AUTO_EXPOSURE_CONTROL = 1,
    RGB_GET_AUTO_EXPOSURE_GET_POINT = 3,
    RGB_GET_AUTO_EXPOSURE_CONTROL_DAMP_OVER = 4,
    RGB_GET_AUTO_EXPOSURE_CONTROL_DAMP_UNDER = 5,
    RGB_GET_AUTO_EXPOSURE_CONTROL_TOLERANCE = 6,
    RGB_GET_MANUAL_GAIN = 7,
    RGB_GET_INTEGRATION_TIME = 8,
    RGB_GET_AUTO_EXPOSURE_WEIGHT = 9,

    // AWB
    RGB_GET_AUTO_WHITE_BALANCE = 10,
    RGB_GET_AUTO_WHITE_BALANCE_MODE = 12,
    RGB_GET_AUTO_WHITE_BALANCE_INDEX = 13,
    RGB_GET_AUTO_WHITE_BALANCE_DAMPING = 14,

    // WB
    RGB_GET_WHITE_BALANCE_CC_MATRIX = 15,
    RGB_GET_WHITE_BALANCE_OFFGET_R = 16,
    RGB_GET_WHITE_BALANCE_OFFSET_G = 17,
    RGB_GET_WHITE_BALANCE_OFFSET_B = 18,
    RGB_GET_WHITE_BALANCE_GAIN_R = 19,
    RGB_GET_WHITE_BALANCE_GAIN_GR = 20,
    RGB_GET_WHITE_BALANCE_GAIN_GB = 21,
    RGB_GET_WHITE_BALANCE_GAIN_B = 22,

    // DPF
    RGB_GET_DENOISING_PREFILTER = 23,

    // BLS
    RGB_GET_BLACK_LEVEL_SUBTRUCTION_R = 24,
    RGB_GET_BLACK_LEVEL_SUBTRUCTION_GR = 25,
    RGB_GET_BLACK_LEVEL_SUBTRUCTION_GB = 26,
    RGB_GET_BLACK_LEVEL_SUBTRUCTION_B = 27,

    // LSC
    RGB_GET_LENS_SHADING_CORRECTION = 28,

    // DWE
    RGB_GET_DEWARP_BYPASS = 29,
    RGB_GET_HFLIP = 30,
    RGB_GET_VFLIP = 31,

    // GAMMA
    RGB_GET_GAMMA = 32,

    // CPROC
    RGB_GET_COLOR_PROCESSING = 33,
    RGB_GET_COLOR_SPACE = 34,
    RGB_GET_BRIGHTNESS = 35,
    RGB_GET_CONTRAST = 36,
    RGB_GET_SATURATION = 37,
    RGB_GET_HUE = 38,

    // DPC
    RGB_GET_DEFECT_PIXEL_CLUSTER_CORRECTION = 39
};

class CameraCtrl
{
  public:
    virtual void setControlInt(int ctrl, int val) = 0;
    virtual int getControlInt(int ctrl, int & r_val) = 0;

    virtual void setControlFloat(int ctrl, float val) = 0;
    virtual int getControlFloat(int ctrl, float & r_val) = 0;

    virtual void setControlBool(int ctrl, bool val) = 0;
    virtual int getControlBool(int ctrl, bool & r_val) = 0;

    virtual void setControlFloatArray(int ctrl, float * vals, int arr_len) = 0;
    virtual std::vector<float> getControlFloatArray(int ctrl, int arr_len) = 0;
    virtual ~CameraCtrl() = default;
};

class CameraCtrlExtern : public CameraCtrl
{
  public:
    CameraCtrlExtern();
    ~CameraCtrlExtern();

    void setControlInt(int ctrl, int val) override;
    int getControlInt(int ctrl, int & r_val) override;

    void setControlFloat(int ctrl, float val) override;
    int getControlFloat(int ctrl, float & r_val) override;

    void setControlBool(int ctrl, bool val) override;
    int getControlBool(int ctrl, bool & r_val) override;

    void setControlFloatArray(int ctrl, float * vals, int arr_len) override;
    std::vector<float> getControlFloatArray(int ctrl, int arr_len) override;

  private:
    // CIS Protocol device
    int fd_;
    bool ctrl_ok = true;

    void configSerial();

    int readCispVal(std::string & out_val, int ctrl, uint8_t byte_len);
};

class CameraCtrlIntern : public CameraCtrl
{
  public:
    CameraCtrlIntern() {}

    void setControlInt(int ctrl, int val) override;
    int getControlInt(int ctrl, int & r_val) override;

    void setControlFloat(int ctrl, float val) override;
    int getControlFloat(int ctrl, float & r_val) override;

    void setControlBool(int ctrl, bool val) override;
    int getControlBool(int ctrl, bool & r_val) override;

    void setControlFloatArray(int ctrl, float * vals, int arr_len) override;
    std::vector<float> getControlFloatArray(int ctrl, int arr_len) override;
};

// class ToFControl

// class CameraCtrlToF : CameraCtrl
// {
// public:
//     void setHistoryThreshold(float val);
//     void setHistoryLength(int val);
//     void setMinReflectance(float val);
//     void setMinConfidence(float val);
//     void setKillFlyingDelta(float val);
//     void setIntegrationTime(int val);

//     float getHistoryThreshold();
//     int getHistoryLength();
//     float getMinReflectance();
//     float getMinConfidence();
//     float getKillFlyingDelta();
//     int getIntegrationTime();
// };
}  // namespace cis_scm
#endif  // CIS_SCM__CONTROLS_HPP_
