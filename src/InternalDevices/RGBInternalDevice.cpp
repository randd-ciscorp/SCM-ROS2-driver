#include "cis_scm/InternalDevice.hpp"

#include "scmcap.h"

namespace cis_scm{
namespace internal{

RGBInternalDevice::RGBInternalDevice(cis::CameraAR0234* camInputDev) : camInputDevice_(camInputDev) {

    DevInfo devInfo {};
    devInfo.width = camInputDevice_->get_width();
    devInfo.height = camInputDevice_->get_height();
    devInfo_ = devInfo;
}

RGBInternalDevice::RGBInternalDevice(cis::CameraIMX715* camInputDev) : camInputDevice_(camInputDev) {

    DevInfo devInfo {};
    devInfo.width = camInputDevice_->get_width();
    devInfo.height = camInputDevice_->get_height();
    devInfo_ = devInfo;
}

RGBInternalDevice::~RGBInternalDevice()
{
    delete camInputDevice_;
}

int RGBInternalDevice::connect()
{
    camInputDevice_->open("/dev/video2");

    if (!camInputDevice_->connect())
    {
        if (!camInputDevice_->stream_on())
        {
            isStreamOn_ = true;
            return 0;
        }
        else
        {
            perror("Could not start streaming");
        }
    }
    else
    {
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

int RGBInternalDevice::getData(uint8_t* data)
{
    if (camInputDevice_->streaming())
    {
        if (cis::connection_handler(&camEvent_, camInputDevice_))
        {
            auto inputBuf = camInputDevice_->pop();
            memcpy(data, inputBuf->data, devInfo_.width * devInfo_.height * 2);
            camInputDevice_->push(inputBuf);
            return 0;
        }
    }
    else
    {
        isStreamOn_ = false;
    }
    return -1;
}
} // namespace internal
} // namespace cis_scm
