#include "cis_scm/InternalDevice.hpp"

#include <fstream>

#include "camera.h"
#include "tof.h"

namespace cis_scm{
namespace internal{
InternalDevice::InternalDevice()
{
    bufInd_ = 0;
    buffers_ = nullptr;
}

InternalDevice::~InternalDevice()
{
}

DevInfo InternalDevice::getInfo() const
{
    return devInfo_;
}
}
}
