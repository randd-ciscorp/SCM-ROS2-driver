// Copyright 2025 CIS Corporation
#include "cis_scm/InternalDevice.hpp"

#include "scmcap.h"
#include "tof.h"

namespace cis_scm
{
namespace internal
{

ToFInternalDevice::ToFInternalDevice(const std::string & path)
{
    DevInfo devInfo{};
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

ToFInternalDevice::~ToFInternalDevice() { disconnect(); }

int ToFInternalDevice::connect()
{
    camInputDevice_.open("/dev/mxc_isi.0.capture");
    if (!camInputDevice_.connect()) {
        if (!camInputDevice_.stream_on()) {
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

void ToFInternalDevice::disconnect()
{
    camInputDevice_.stream_off();
    camInputDevice_.disconnect();
}

int ToFInternalDevice::getData(uint8_t * data)
{
    if (camInputDevice_.streaming()) {
        if (cis::connection_handler(&camEvent_, &camInputDevice_)) {
            auto inputBuf = camInputDevice_.pop();

            cis::tof_calc_distance_dual(
                reinterpret_cast<char *>(inputBuf->data), reinterpret_cast<char *>(data),
                &tofParams_, &calibData_);

            camInputDevice_.push(inputBuf);
            return 0;
        }
    } else {
        isStreamOn_ = false;
    }
    return -1;
}
}  // namespace internal
}  // namespace cis_scm
