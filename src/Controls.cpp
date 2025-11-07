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

#include "cis_scm/Controls.hpp"

#include <fcntl.h>
#include <sys/types.h>
#include <termio.h>
#include <unistd.h>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

namespace cis_scm
{

CameraCtrlExtern::CameraCtrlExtern()
{
    fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        perror("Error opening CIS Protocol device");
        throw std::runtime_error("CameraCtrlExtern init failed");
    }

    configSerial();
}

void CameraCtrlExtern::configSerial()
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
        perror("Error from tcgetattr");
    }

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;                          // 1 stop bit
    tty.c_cflag &= ~CSIZE;                           // Clear current character size mask
    tty.c_cflag |= CS8;                              // 8 data bits
    tty.c_cflag &= ~CRTSCTS;                         // No hardware flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // No software flow control
    tty.c_oflag &= ~OPOST;                           // No output processing
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non-canonical mode

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
    }
}

CameraCtrlExtern::~CameraCtrlExtern() { close(fd_); }

void CameraCtrlExtern::setControlInt(int ctrl, int val)
{
    std::stringstream ss;
    ss << "SU " << ctrl << " " << val << "\r\n";
    std::string cmd = ss.str();
    std::cout << cmd << std::endl;
    if (write(fd_, cmd.c_str(), cmd.length()) <= 0) {
        std::cerr << "Failed to write to /dev/ttymxc0" << std::endl;
    }
    tcdrain(fd_);
}

void CameraCtrlExtern::setControlFloat(int ctrl, float val)
{
    std::stringstream ss;
    ss << "SU " << ctrl << " " << val << "\r\n";
    std::string cmd = ss.str();
    std::cout << cmd << std::endl;
    if (write(fd_, cmd.c_str(), cmd.length()) <= 0) {
        std::cerr << "Failed to write to /dev/ttymxc0" << std::endl;
    }
    tcdrain(fd_);
}

void CameraCtrlExtern::setControlBool(int ctrl, bool val)
{
    std::stringstream ss;
    ss << "SU " << ctrl << " " << val << "\r\n";
    std::string cmd = ss.str();
    std::cout << cmd << std::endl;
    if (write(fd_, cmd.c_str(), cmd.length()) <= 0) {
        std::cerr << "Failed to write to /dev/ttymxc0" << std::endl;
    }
    tcdrain(fd_);
}

void CameraCtrlExtern::setControlFloatArray(int ctrl, float * vals, int arr_len)
{
    std::stringstream ss;
    ss << "SU " << ctrl;
    for (int i = 0; i < arr_len; i++) {
        ss << " " << vals[i];
    }
    ss << "\r\n";

    std::string cmd = ss.str();
    std::cout << cmd << std::endl;
    if (write(fd_, cmd.c_str(), cmd.length()) <= 0) {
        std::cerr << "Failed to write to /dev/ttymxc0" << std::endl;
    }
    tcdrain(fd_);
}

int CameraCtrlExtern::readCispVal(std::string & out_val, int ctrl_id, uint8_t byte_len)
{
    // ctrl value query
    std::stringstream ss;
    ss << "GU " << ctrl_id << "\r";
    std::string cmd = ss.str();

    // get ctrl value
    char r_buf[256];
    memset(r_buf, 0, sizeof(r_buf));
    tcflush(fd_, TCIOFLUSH);
    write(fd_, cmd.c_str(), cmd.length());

    // Data query and transfer takes time --> Wait to receive all data
    usleep(20000);
    // tcflush(fd_, TCOFLUSH);

    int r = read(fd_, &r_buf, byte_len);
    // tcdrain(fd_);
    if (r > 0) {
        out_val.append(r_buf, r_buf + r);
    }

    // printf("Read %d: \"%s\"\n", r, out_val.c_str());
    return !out_val.empty();
}

int CameraCtrlExtern::getControlInt(int ctrl, int & r_val)
{
    std::string r_str;
    int r = readCispVal(r_str, ctrl, 255);
    if (r) {
        // Having two \n before ctrl value
        size_t val_pos = r_str.find_first_of("\n");
        size_t val_pos_end = r_str.find_first_of("\n", val_pos + 2);
        try {
            r_val = std::stoi(r_str.substr(val_pos, val_pos_end - val_pos));
            return 0;
        } catch (std::exception & e) {
            // Could't find right control value from serial transfer
        }
    }
    return -1;
}

int CameraCtrlExtern::getControlFloat(int ctrl, float & r_val)
{
    std::string r_str;
    int r = readCispVal(r_str, ctrl, 255);
    if (r) {
        // Having two \n before ctrl value
        size_t val_pos = r_str.find_first_of("\n");
        size_t val_pos_end = r_str.find_first_of("\n", val_pos + 2);
        try {
            r_val = std::stof(r_str.substr(val_pos, val_pos_end - val_pos));
            return 0;
        } catch (std::exception & e) {
            // Could't find right control value from serial transfer
        }
    }
    return -1;
}

int CameraCtrlExtern::getControlBool(int ctrl, bool & r_val)
{
    std::string r_str;
    int r = readCispVal(r_str, ctrl, 255);
    if (r) {
        // Having two \n before ctrl value
        size_t val_pos = r_str.find_first_of("\n");
        size_t val_pos_end = r_str.find_first_of("\n", val_pos + 2);
        try {
            r_val = std::stoi(r_str.substr(val_pos, val_pos_end - val_pos));
            return 0;
        } catch (std::exception & e) {
            // Could't find right control value from serial transfer
        }
    }
    return -1;
}

int CameraCtrlExtern::getControlFloatArray(int ctrl, std::vector<float> & r_vals, int arr_len)
{
    std::string r_str;
    int r = readCispVal(r_str, ctrl, 255);
    if (r) {
        // Having two \n before ctrl value
        size_t val_pos = r_str.find_first_of("\n");
        size_t val_pos_end = r_str.find_first_of("\n", val_pos + 2);
        r_vals = std::vector<float>(10);
        try {
            auto r_sub_str = r_str.substr(val_pos, val_pos_end - val_pos);
            int st_pos = 0;
            for (int i = 0; i < arr_len; i++) {
                size_t space_pos = r_sub_str.find(" ", st_pos);
                r_vals[i] = std::stof(r_sub_str.substr(st_pos, space_pos - st_pos));
                st_pos = space_pos + 1;
            }
            return 0;
        } catch (std::exception & e) {
            // Could't find right control value from serial transfer
        }
    }
    return -1;
}
}  // namespace cis_scm
