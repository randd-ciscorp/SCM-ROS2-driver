#include "cis_scm/InternalDevice.hpp"

namespace cis_scm{
namespace internal{

RGBInternalDevice::RGBInternalDevice(cis::CameraAR0234& camInputDev) {
    camInputDevice_ = &camInputDev;

    DevInfo devInfo {};
    devInfo.width = camInputDevice_->get_width();
    devInfo.height = camInputDevice_->get_height();
    devInfo_ = devInfo;
}

RGBInternalDevice::RGBInternalDevice(cis::CameraIMX715& camInputDev) {
    camInputDevice_ = &camInputDev;

    DevInfo devInfo {};
    devInfo.width = camInputDevice_->get_width();
    devInfo.height = camInputDevice_->get_height();
    devInfo_ = devInfo;
}

int RGBInternalDevice::connect()
{
    printf("Conneting...");
    int r = camInputDevice_->connect();
    r = camInputDevice_->stream_on();
    if (!r)
    {
        isStreamOn_ = true;
    }
    else
    {
        perror("Connection failed");
    }

    return r;
}

void RGBInternalDevice::disconnect()
{
    camInputDevice_->disconnect();
}

int RGBInternalDevice::getData(uint8_t* data)
{
    camEvent_.wait(isStreamOn_);
    bool canRead = camEvent_.can_read(camInputDevice_->fd());
    if (isStreamOn_ & canRead)
    {
        auto inputBuf = camInputDevice_->pop();
        memcpy(data, inputBuf->data, devInfo_.width * devInfo_.height * 2);
        camInputDevice_->push(inputBuf);
        return 0;
    }
    return -1;
}
} // namespace internal
} // namespace cis_scm
