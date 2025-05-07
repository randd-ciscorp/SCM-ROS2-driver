#include "tof1_driver/Device.h"

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

int Device::errnoExit(const char *s) const{
    fprintf(stderr, "'%s': '%d, %s \n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
    return errno;
}

Device::Device()
{
    fd_ = -1;
    data_ = nullptr;
    buffers_ = nullptr;
    buffers_ = nullptr;
    bufInd_ = 0;
}

Device::~Device()
{
    disconnect();
}

void Device::initMmap(){

    // Request buffer
    struct v4l2_requestbuffers req;
    req.count = N_CAP_BUF;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req))
    {
        errnoExit("Request buffers");
    }

    // MMAP
    buffers_ = (struct RequestBuffer*)calloc(req.count, sizeof(RequestBuffer));
    for (int bufInd = 0; bufInd < N_CAP_BUF; bufInd++)
    {
        struct v4l2_buffer buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = bufInd;
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
        {
            errnoExit("Buffer query failed");
        }

        buffers_[bufInd].length = buf.length;
        buffers_[bufInd].data = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_ ,buf.m.offset);
        if (buffers_[bufInd].data == MAP_FAILED)
        {
            errnoExit("MMAP failed");
        }

        bufInd = buf.index;
        
        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0)
        {
            errnoExit("QBUF failed");
        }
    }
}

int Device::connect(int height, int width)
{
    const char* deviceName = "/dev/video0";

    // Open
    fd_ = open(deviceName, O_RDWR | O_NONBLOCK);
    if (fd_ < 0)
    {
        errnoExit("Opening");
    }
    
    // Format
    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0)
    {
        errnoExit("Format setting");
    }

    initMmap();

    // StreamON
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type))
    {
        errnoExit("StreamON");
    }
    isStreamOn_ = true;
    
    return S_OK;
}

void Device::disconnect(){
    if (isStreamOn_)
    {
        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
        isStreamOn_ = false;
    }

    if (buffers_)
    {
        for (int bufInd = 0; bufInd < N_CAP_BUF; bufInd++)
        {
            munmap(buffers_[bufInd].data, buffers_[bufInd].length);
        }
        free(buffers_);
        buffers_ = nullptr;
    }

    close(fd_);
}

DevInfo Device::getInfo() const
{
    DevInfo devInfo;

    // Capability
    struct v4l2_capability cap;
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0)
    {
        errnoExit("Capability query");
    }
    printf("Driver: %s \n", cap.driver);
    printf("Card: %s \n", cap.card);
    printf("  Version: %u.%u.%u",
        (cap.version >> 16) & 0xFF,
        (cap.version >> 8)  & 0xFF,
        (cap.version)       & 0xFF);

    
    struct v4l2_format fmt {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_G_FMT, &fmt) < 0)
    {
        errnoExit("Get device format");
        return DevInfo();
    }    

    devInfo.devName = std::string(reinterpret_cast<char*>(cap.card));
    devInfo.driverVers = std::string(reinterpret_cast<char*>(cap.driver));
    devInfo.sn = std::string(reinterpret_cast<char*>(cap.bus_info));
    devInfo.width = fmt.fmt.pix.width;
    devInfo.height = fmt.fmt.pix.height;

    return devInfo;
}

int Device::getData(float* data){
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    if (select(fd_ + 1, &fds, NULL, NULL, NULL) <= 0)
    {
        errnoExit("Frame capture timeout");
    }

    if(FD_ISSET(fd_, &fds)){

        struct v4l2_buffer in_buf{};
        in_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        in_buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd_, VIDIOC_DQBUF, &in_buf) < 0)
        {
            errnoExit("DQbuf");
        }
        
        memcpy((void*)data, buffers_[in_buf.index].data, buffers_[in_buf.index].length);

        if (xioctl(fd_, VIDIOC_QBUF, &in_buf) < 0)
        {
            errnoExit("QBuf");
        }

        return 1;
    }

    return -1;
}

bool Device::isConnected() const
{
    return isStreamOn_;
}