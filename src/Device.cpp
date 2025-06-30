#include "cis_scm/Device.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include <errno.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

namespace cis_scm {

Device::Device()
{
    fd_ = -1;
    buffers_ = nullptr;
    bufInd_ = 0;
    isStreamOn_ = false;
}

Device::~Device()
{
}

bool Device::isConnected() const
{
    return isStreamOn_;
}
}
