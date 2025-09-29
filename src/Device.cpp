// Copyright 2025 CIS Corporation
#include "cis_scm/Device.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

namespace cis_scm
{

Device::Device()
{
    fd_ = -1;
    buffers_ = nullptr;
    bufInd_ = 0;
    isStreamOn_ = false;
}

Device::~Device() {}

bool Device::isConnected() const { return isStreamOn_; }
}  // namespace cis_scm
