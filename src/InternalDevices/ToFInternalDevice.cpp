#include "cis_scm/InternalDevice.hpp"

#include <fstream>

#include "tof.h"

namespace cis_scm{
namespace internal{
ToFInternalDevice::ToFInternalDevice(const std::string &path)
{
    DevInfo devInfo {};
    devInfo.width = camInputDevice_.get_width();
    devInfo.height = camInputDevice_.get_height();
    devInfo_ = devInfo;

    printf("Cam inititalised \n");

    // ToF calibs loading
    cis::init_calibdata(&calibData_);

    tofParams_ = cis::ToFParam();
    tofParams_.MIN_REFLECTANCE = 7.5f;
    tofParams_.MIN_CONFIDENCE = 7.5f;
    tofParams_.KILL_FLYING_DELTA = 0.03f;
}

ToFInternalDevice::~ToFInternalDevice()
{
    disconnect();
}

int ToFInternalDevice::connect()
{
    printf("Conneting...");
    camInputDevice_.open("/dev/mxc_isi.0.capture");
    if (camInputDevice_.streaming())
    {
        printf("Cam inititalised \n");
        isStreamOn_ = true;
        return 0;
    }
    else
    {
        perror("Connection failed");
        return -1;
    }
}

void ToFInternalDevice::disconnect()
{
    camInputDevice_.disconnect();
}

int ToFInternalDevice::getData(uint8_t* data)
{
    if (camInputDevice_.streaming())
    {
        auto inputBuf = camInputDevice_.pop();

        cis::tof_calc_distance_dual((char*) inputBuf->data, (char*) data, &tofParams_, &calibData_);

        camInputDevice_.push(inputBuf);
        return 0;
    }
    else
    {
        isStreamOn_ = false;
    }
    return -1;
}
}
}
