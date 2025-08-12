#include "cis_scm/Controls.hpp"

namespace cis_scm
{
void CameraCtrlExtern::setControlInt(int ctrl, int val)
{
    std::cout << "SU " << ctrl << " " << val << std::endl;
}

void CameraCtrlExtern::setControlFloat(int ctrl, float val)
{
    std::cout << "SU " << ctrl << " " << val << std::endl;
}

int CameraCtrlExtern::getControlInt(int ctrl)
{
    int ret_val = 0;
    std::cout << "GU " << ctrl << std::endl;
    return ret_val;
}

float CameraCtrlExtern::getControlFloat(int ctrl)
{
    float ret_val = 0.;
    std::cout << "GU " << ctrl << std::endl;
    return ret_val;
}

void CameraCtrlIntern::setControlInt(int ctrl, int val)
{
    std::cout << "SU " << ctrl << " " << val << std::endl;
}

void CameraCtrlIntern::setControlFloat(int ctrl, float val)
{
    std::cout << "SU " << ctrl << " " << val << std::endl;
}

int CameraCtrlIntern::getControlInt(int ctrl)
{
    int ret_val = 0;
    std::cout << "GU " << ctrl << std::endl;
    return ret_val;
}

float CameraCtrlIntern::getControlFloat(int ctrl)
{
    float ret_val = 0.;
    std::cout << "GU " << ctrl << std::endl;
    return ret_val;
}
} // namespace cis_scm


