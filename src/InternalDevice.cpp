#include "cis_scm/InternalDevice.hpp"

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
