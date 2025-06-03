#include "cis_scm/InternalDevice.hpp"

#include <fstream>

#include "camera.h"
#include "tof.h"

namespace cis_scm{

InternalDevice::InternalDevice()
{
    fd_ = -1;
    data_ = nullptr;
    buffers_ = nullptr;
    buffers_ = nullptr;
    bufInd_ = 0;
    isStreamOn_ = false;
}

InternalDevice::~InternalDevice()
{
    camInputDevice_.stream_off();
    disconnect();
}

void InternalDevice::init(const std::string& path)
{
    printf("Cam inititalisation");
    camInputDevice_.open();
    camInputDevice_.init(&camEvent_);
    printf("Cam inititalised");

    cis::init_calibdata(&calibData_, path);
    std::ifstream ifs(path + "tof_param.txt");
	cis::ToFParam param;
	ifs >> param.MIN_REFLECTANCE;
	ifs >> param.MIN_CONFIDENCE;
	ifs >> param.KILL_FLYING_DELTA;
	ifs.close();
}

int InternalDevice::connect(const std::string& path)
{
    init(path);
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

void InternalDevice::disconnect()
{
    camInputDevice_.disconnect();
}

bool InternalDevice::isConnected() const
{
    return isStreamOn_;
}

DevInfo InternalDevice::getInfo() const
{
    DevInfo devInfo {};
    devInfo.width = 640;
    devInfo.height = 480;

    return devInfo;
}

int InternalDevice::getData(uint8_t* data)
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
