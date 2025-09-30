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

#include "cis_scm/InternalDevice.hpp"
#include "tof.h"

namespace cis_scm
{

namespace internal
{

RGBDInternalDevice::RGBDInternalDevice(const std::string & path, bool align)
: tofIntDev(path), isAligned(align)
{
    rgbIntDev = RGBInternalDevice(new cis::CameraAR0234());
}

int RGBDInternalDevice::connect()
{
    int ret = tofIntDev.connect();
    ret |= rgbIntDev.connect();
    return ret;
}

void RGBDInternalDevice::disconnect()
{
    tofIntDev.disconnect();
    rgbIntDev.disconnect();
}

int RGBDInternalDevice::getData(uint8_t * outData)
{
    std::vector<float> tofData =
        std::vector<float>(tofIntDev.getInfo().width * tofIntDev.getInfo().height);
    tofIntDev.getData(reinterpret_cast<uint8_t *>(tofData.data()));

    std::vector<uint8_t> rgbData =
        std::vector<uint8_t>(rgbIntDev.getInfo().width * rgbIntDev.getInfo().height);
    rgbIntDev.getData(rgbData.data());

    // std::vector<uint8_t> outData = std::vector<uint8_t>(tofData.size() * 4 * rgbData.size());

    outData = new uint8_t[tofData.size() * sizeof(float) * rgbData.size()];

    if (isAligned) {
        cis::ToFParam tofParams = tofIntDev.getToFParams();
        cis::CalibData tofCalib = tofIntDev.getCalibData();
        cis::FusionData fusionData;
        cis::init_fusiondata(&fusionData);
        cis::rgbd_fusion(
            reinterpret_cast<char *>(tofData.data()), reinterpret_cast<char *>(rgbData.data()),
            reinterpret_cast<char *>(outData), &tofParams, &tofCalib, &fusionData);
    } else {
        memcpy(outData, tofData.data(), tofData.size() * sizeof(float));
        memcpy(outData + tofData.size() * sizeof(float), rgbData.data(), rgbData.size());
    }

    return 0;
}

}  // namespace internal
}  // namespace cis_scm
