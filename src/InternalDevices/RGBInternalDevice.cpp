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
#include "scmcap.h"

namespace cis_scm
{
namespace internal
{

RGBInternalDevice::RGBInternalDevice(cis::CameraAR0234 * camInputDev) : camInputDevice_(camInputDev)
{
    DevInfo devInfo{};
    devInfo.width = camInputDevice_->get_width();
    devInfo.height = camInputDevice_->get_height();
    devInfo_ = devInfo;
}

RGBInternalDevice::RGBInternalDevice(cis::CameraIMX715 * camInputDev) : camInputDevice_(camInputDev)
{
    DevInfo devInfo{};
    devInfo.width = camInputDevice_->get_width();
    devInfo.height = camInputDevice_->get_height();
    devInfo_ = devInfo;
}

RGBInternalDevice::~RGBInternalDevice() { delete camInputDevice_; }

int RGBInternalDevice::connect()
{
    camInputDevice_->open("/dev/video2");

    if (!camInputDevice_->connect()) {
        if (!camInputDevice_->stream_on()) {
            isStreamOn_ = true;
            return 0;
        } else {
            perror("Could not start streaming");
        }
    } else {
        perror("Connection failed");
    }
    isStreamOn_ = false;
    return -1;
}

void RGBInternalDevice::disconnect()
{
    camInputDevice_->stream_off();
    camInputDevice_->disconnect();
}

int RGBInternalDevice::getData(uint8_t * data)
{
    if (camInputDevice_->streaming()) {
        if (cis::connection_handler(&camEvent_, camInputDevice_)) {
            auto inputBuf = camInputDevice_->pop();
            memcpy(data, inputBuf->data, devInfo_.width * devInfo_.height * 2);
            camInputDevice_->push(inputBuf);
            return 0;
        }
    } else {
        isStreamOn_ = false;
    }
    return -1;
}
}  // namespace internal
}  // namespace cis_scm
