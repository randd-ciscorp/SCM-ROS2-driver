#include "tof1_driver/Device.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/mman.h>

#include <linux/videodev2.h>

int Device::xioctl(int fd, int req, void* arg){
    int r;
	do {
		r = ioctl(fd, req, arg);
	} while(r == -1 && errno == EINTR);
	return r;
}

int Device::errno_exit(const char *s){
    fprintf(stderr, "'%s': '%d, %s \n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
    return errno;
}

Device::Device()
{
    thrdParams_.thread_id = 0;
    thrdParams_.fd = -1;
    thrdParams_.data_len = 0;
    thrdParams_.data = nullptr;
    thrdParams_.buffers = nullptr;
    thrdParams_.run = false;
    thrdParams_.received = false;
    thrdParams_.buffers = nullptr;
    thrdParams_.bufInd = 0;
}

Device::~Device()
{
    Disconnect();
}

void Device::init_mmap(){

    // Request buffer
    struct v4l2_requestbuffers req;
    req.count = N_IMAGES;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(FD, VIDIOC_REQBUFS, &req))
    {
        errno_exit("Request buffers");
    }

    // MMAP
    thrdParams_.buffers = (struct RequestBuffer*)calloc(req.count, sizeof(RequestBuffer));
    for (int bufInd = 0; bufInd < req.count; bufInd++)
    {
        struct v4l2_buffer buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = bufInd;
        if (xioctl(FD, VIDIOC_QUERYBUF, &buf) < 0)
        {
            errno_exit("Buffer query failed");
        }

        thrdParams_.buffers[bufInd].length = buf.length;
        thrdParams_.buffers[bufInd].data = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, FD ,buf.m.offset);
        if (thrdParams_.buffers[bufInd].data == MAP_FAILED)
        {
            errno_exit("MMAP failed");
        }

        thrdParams_.bufInd = buf.index;
        
        if (xioctl(FD, VIDIOC_QBUF, &buf) < 0)
        {
            errno_exit("QBUF failed");
        }
    }
}

int Device::Connect(const char* serial_number, int height, int width)
{
    CInstanceId instanceId;
    instanceId.DeviceName = "/dev/video0";

    if (serial_number = NULL)
    {
        return E_POINTER;
    }

    // Open
    FD = open(instanceId.DeviceName.data(), O_RDWR | O_NONBLOCK);
    if (FD < 0)
    {
        errno_exit("Opening");
    }
    
    // Capability
    struct v4l2_capability cap;
    if (xioctl(FD, VIDIOC_QUERYCAP, &cap) < 0)
    {
        errno_exit("Capability query");
    }
    printf("Driver: %s \n", cap.driver);
    printf("Card: %s \n", cap.card);
    printf("  Version: %u.%u.%u",
        (cap.version >> 16) & 0xFF,
        (cap.version >> 8)  & 0xFF,
        (cap.version)       & 0xFF);
    

    // Format
    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(FD, VIDIOC_S_FMT, &fmt) < 0)
    {
        errno_exit("Format setting");
    }

    init_mmap();

    // StreamON
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(FD, VIDIOC_STREAMON, &type))
    {
        errno_exit("StreamON");
    }
    streamOn_ = true;
    
    return S_OK;
}

void Device::Disconnect(){
    if (streamOn_)
    {
        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(FD, VIDIOC_STREAMOFF, &type);
        streamOn_ = false;
    }

    if (thrdParams_.buffers)
    {
        for (int bufInd = 0; bufInd < N_IMAGES; bufInd++)
        {
            munmap(thrdParams_.buffers[bufInd].data, thrdParams_.buffers[bufInd].length);
        }
        free(thrdParams_.buffers);
        thrdParams_.buffers = nullptr;
    }

    close(FD);
}

int Device::GetDeviceInfo(std::vector<CInstanceId>* pInstanceIdList){
    return 1;
}

void* Device::GetFrameData(){
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(FD, &fds);

    struct timeval tv{};
    tv.tv_sec = 2;
    if (select(FD + 1, &fds, NULL, NULL, NULL) <= 0)
    {
        errno_exit("Frame capture timeout");
    }

    if(FD_ISSET(FD, &fds)){

        struct v4l2_buffer in_buf{};
        in_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        in_buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(FD, VIDIOC_DQBUF, &in_buf) < 0)
        {
            errno_exit("DQbuf");
        }

        if (xioctl(FD, VIDIOC_QBUF, &in_buf) < 0)
        {
            errno_exit("QBuf");
        }

        return thrdParams_.buffers[in_buf.index].data;
    }
}

int Device::GetData(float* data){
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(FD, &fds);

    struct timeval tv{};
    tv.tv_sec = 2;
    if (select(FD + 1, &fds, NULL, NULL, NULL) <= 0)
    {
        errno_exit("Frame capture timeout");
    }

    if(FD_ISSET(FD, &fds)){

        struct v4l2_buffer in_buf{};
        in_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        in_buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(FD, VIDIOC_DQBUF, &in_buf) < 0)
        {
            errno_exit("DQbuf");
        }
        
        memcpy((void*)data, thrdParams_.buffers[in_buf.index].data, thrdParams_.buffers[in_buf.index].length);

        if (xioctl(FD, VIDIOC_QBUF, &in_buf) < 0)
        {
            errno_exit("QBuf");
        }

        return 1;
    }
}

bool Device::IsConnected()
{
    return streamOn_;
}