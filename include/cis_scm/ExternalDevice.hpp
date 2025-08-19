#ifndef EXTERNAL_DEVICE_HPP
#define EXTERNAL_DEVICE_HPP

#include <sys/ioctl.h>
#include <string>

#include "Device.h"

namespace cis_scm {


class ExternalDevice : public Device
{
public:
    ExternalDevice();
    ~ExternalDevice();

    int connect(int width, int height);
    void disconnect();

    DevInfo getInfo() const;
    int getData(uint8_t* data);

private:
    void initMmap();
};

}
#endif
