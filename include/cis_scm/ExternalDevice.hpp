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

#ifndef CIS_SCM__EXTERNALDEVICE_HPP_
#define CIS_SCM__EXTERNALDEVICE_HPP_

#include <sys/ioctl.h>

#include "Device.h"

namespace cis_scm
{

inline constexpr std::string_view cis_dev_name = "SCM Series: UVC Camera";

/**
 * @brief Implementation of Device for external (through UVC) capture
 */
class ExternalDevice : public Device
{
  public:
    ExternalDevice();
    ~ExternalDevice();

    /**
   * @brief Connect to the external device and configure resolution.
   * @param width Frame width in pixels.
   * @param height Frame height in pixels.
   * @return 0 on success, errno-compatible error code on failure.
   */
    int connect(int width, int height);

    /**
   * @brief Disconnect and release all resources.
   */
    void disconnect();

    /**
   * @brief Get current device information.
   */
    DevInfo getInfo() const;

    /**
   * @brief Retrieve one frame of data from the device.
   */
    int getData(uint8_t * data);

  private:
    void initMmap();
    /**
    * @brief Find the video device created by SCM's uvc, and open id to fd_.
    * @return 0: SCM video dev found and opened  | -1: SCM video dev not found or opened. 
    */
    int openVideoDev();
};

}  // namespace cis_scm
#endif  // CIS_SCM__EXTERNALDEVICE_HPP_
