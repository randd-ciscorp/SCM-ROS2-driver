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

#ifndef CIS_SCM__INTERNALDEVICE_HPP_
#define CIS_SCM__INTERNALDEVICE_HPP_

#include <sys/ioctl.h>

#include <cerrno>
#include <cstdint>
#include <string>

#include "Device.h"
#include "scmcap.h"
#include "tof.h"

#define N_CAP_BUF 4

// Error type
#define S_OK 0
#define E_POINTER EFAULT
#define E_FAIL ENODATA
#define E_INVALIDARG EINVAL
#define E_OUTOFMEMORY ENOMEM
#define E_HANDLE ENODATA

namespace cis_scm
{

namespace internal
{

class InternalDevice : public Device
{
  protected:
    InternalDevice();
    ~InternalDevice();

  public:
    virtual int connect() = 0;
    virtual void disconnect() = 0;

    DevInfo getInfo() const;
    int getData(uint8_t * data) override = 0;

  protected:
    DevInfo devInfo_;
    cis::CameraEvent camEvent_;

    unsigned int bufInd_;
    struct RequestBuffer * buffers_;
};

class ToFInternalDevice : public InternalDevice
{
  public:
    ToFInternalDevice() {}
    explicit ToFInternalDevice(const std::string & path);
    ~ToFInternalDevice();

    int connect() override;
    void disconnect() override;

    int getData(uint8_t * data) override;

    cis::CalibData getCalibData() { return calibData_; }
    cis::ToFParam getToFParams() { return tofParams_; }

  private:
    cis::CameraIMX570 camInputDevice_;
    cis::CalibData calibData_;
    cis::ToFParam tofParams_;
};

class RGBInternalDevice : public InternalDevice
{
  public:
    RGBInternalDevice() {}
    explicit RGBInternalDevice(cis::CameraAR0234 * camInputDevice);
    explicit RGBInternalDevice(cis::CameraIMX715 * camInputDevice);
    ~RGBInternalDevice();

    int connect() override;
    void disconnect() override;

    int getData(uint8_t * data) override;

  private:
    cis::CameraInputColor * camInputDevice_;
};

class RGBDInternalDevice : public InternalDevice
{
  public:
    RGBDInternalDevice(const std::string & path, bool align);

    int connect() override;
    void disconnect() override;

    int getData(uint8_t * data) override;

  private:
    ToFInternalDevice tofIntDev;
    RGBInternalDevice rgbIntDev;

    bool isAligned;
};
}  // namespace internal
}  // namespace cis_scm

#endif  // CIS_SCM__INTERNALDEVICE_HPP_
