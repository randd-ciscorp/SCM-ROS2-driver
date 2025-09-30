// Copyright 2025 CIS Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cis_scm/InternalDevice.hpp"

namespace cis_scm
{
namespace internal
{
InternalDevice::InternalDevice()
{
    bufInd_ = 0;
    buffers_ = nullptr;
}

InternalDevice::~InternalDevice() {}

DevInfo InternalDevice::getInfo() const { return devInfo_; }
}  // namespace internal
}  // namespace cis_scm
