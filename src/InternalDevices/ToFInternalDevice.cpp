#include "cis_scm/InternalDevice.hpp"

#include <fstream>

#include "tof.h"

namespace cis_scm{
namespace internal{
ToFInternalDevice::ToFInternalDevice(const std::string &path)
{
    DevInfo devInfo {};
    devInfo.width = 640;
    devInfo.height = 480;
    devInfo_ = devInfo;

    printf("Cam inititalisation \n");
    int r = camInputDevice_.open();
    if (!r)
    {
       printf("/dev/video not found \n");
       fflush(stdout);
    }

    camInputDevice_.init(&camEvent_);
    printf("Cam inititalised \n");

    // ToF calibs loading
    cis::init_calibdata(&calibData_, path);
    std::ifstream ifs(path + "tof_param.txt");

    tofParams_ = cis::ToFParam();
	ifs >> tofParams_.MIN_REFLECTANCE;
	ifs >> tofParams_.MIN_CONFIDENCE;
	ifs >> tofParams_.KILL_FLYING_DELTA;
	ifs.close();
}

ToFInternalDevice::~ToFInternalDevice()
{
    disconnect();
}

int ToFInternalDevice::connect()
{
    printf("Conneting...");
    int r = camInputDevice_.connect();
    r = camInputDevice_.stream_on();
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

void ToFInternalDevice::disconnect()
{
    camInputDevice_.disconnect();
}

int ToFInternalDevice::getData(uint8_t* data)
{
    camEvent_.wait(isStreamOn_);
    bool canRead = camEvent_.can_read(camInputDevice_.fd());

    if (isStreamOn_ & canRead)
    {
        auto inputBuf = camInputDevice_.pop();

        cis::tof_calc_distance_dual((char*) inputBuf->data, (char*) data, &tofParams_, &calibData_);


        camInputDevice_.push(inputBuf);
        return 0;
    }
    return -1;
}
}
}
