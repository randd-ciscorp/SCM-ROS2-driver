// Copyright 2025 CIS Corporation
#ifndef CIS_SCM__EXTERNALDEVICE_HPP_
#define CIS_SCM__EXTERNALDEVICE_HPP_

#include <sys/ioctl.h>

#include "Device.h"

namespace cis_scm
{

class ExternalDevice : public Device
{
  public:
    ExternalDevice();
    ~ExternalDevice();

    int connect(int width, int height);
    void disconnect();

    DevInfo getInfo() const;
    int getData(uint8_t * data);

  private:
    void initMmap();
};

}  // namespace cis_scm
#endif  // CIS_SCM__EXTERNALDEVICE_HPP_
