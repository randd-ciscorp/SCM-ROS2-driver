#include "cis_scm/Controls.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termio.h>

namespace cis_scm
{

CameraCtrlExtern::CameraCtrlExtern()
{
    fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0)
    {
        perror("Error opening CIS Protocol device");
        ctrl_ok = false;
    }

    configSerial();
}

void CameraCtrlExtern::configSerial()
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0)
    {
        perror("Error from tcgetattr");
    }

    tty.c_lflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CSIZE;              // Clear current character size mask
    tty.c_cflag |= CS8;                 // 8 data bits
    tty.c_cflag &= ~CRTSCTS;            // No hardware flow control

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
    }
}

CameraCtrlExtern::~CameraCtrlExtern()
{
    close(fd_);
}

void CameraCtrlExtern::setControlInt(int ctrl, int val)
{
    std::stringstream ss;
    ss << "SU " << ctrl << " " << val << "\n";
    std::string cmd = ss.str();
    if(write(fd_, cmd.c_str(), cmd.length()) <= 0)
    {
       std::cerr << "Failed to write to /dev/ttymxc0" << std::endl;
    }
}

void CameraCtrlExtern::setControlFloat(int ctrl, float val)
{
    std::stringstream ss;
    ss << "SU " << ctrl << " " << val << "\n";
    std::string cmd = ss.str();
    std::cout << cmd << std::endl;
    if(write(fd_, cmd.c_str(), cmd.length()) <= 0)
    {
       std::cerr << "Failed to write to /dev/ttymxc0" << std::endl;
    }
}

int CameraCtrlExtern::getControlInt(int ctrl)
{
    // ctrl value query
    std::stringstream ss;
    ss << "GU " << ctrl;
    std::string cmd = ss.str();

    // get ctrl value
    int ret_val = 0;
    char r_buf[sizeof(int)];
    while(read(fd_, r_buf, sizeof(int)) <= (ssize_t)sizeof(int))
    {
        write(fd_, cmd.c_str(), cmd.length());
    }
    return ret_val;
}

float CameraCtrlExtern::getControlFloat(int ctrl)
{
    // ctrl value query
    std::stringstream ss;
    ss << "GU " << ctrl;
    std::string cmd = ss.str();

    // get ctrl value
    int ret_val = 0;
    char r_buf[sizeof(float)];
    while(read(fd_, r_buf, sizeof(float)) <= (ssize_t)sizeof(float))
    {
        write(fd_, cmd.c_str(), cmd.length());
    }
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


