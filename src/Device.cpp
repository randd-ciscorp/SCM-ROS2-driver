#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>

#include "tof1_driver/Device.h"
#include <iostream>

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

Device::Device(/* args */)
{
    m_Param.thread_id = 0;
    m_Param.fd = -1;
    m_Param.data_len = 0;
    m_Param.data = nullptr;
    m_Param.buffers = nullptr;
    m_Param.run = false;
    m_Param.received = false;
    m_Param.buffers = nullptr;
    m_Param.bufInd = 0;
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
    if (xioctl(m_FD, VIDIOC_REQBUFS, &req))
    {
        errno_exit("Request buffers");
    }

    // MMAP
    m_Param.buffers = (struct RequestBuffer*)calloc(req.count, sizeof(RequestBuffer));
    for (int bufInd = 0; bufInd < req.count; bufInd++)
    {
        struct v4l2_buffer buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = bufInd;
        if (xioctl(m_FD, VIDIOC_QUERYBUF, &buf) < 0)
        {
            errno_exit("Buffer query");
        }

        m_Param.buffers[bufInd].length = buf.length;
        m_Param.buffers[bufInd].data = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_FD ,buf.m.offset);
        if (m_Param.buffers[bufInd].data == MAP_FAILED)
        {
            errno_exit("MMAP");
        }

        m_Param.bufInd = buf.index;
        
        if (xioctl(m_FD, VIDIOC_QBUF, &buf) < 0)
        {
            errno_exit("Test QBUF");
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
    m_FD = open(instanceId.DeviceName.data(), O_RDWR | O_NONBLOCK);
    if (m_FD < 0)
    {
        errno_exit("Opening");
    }
    
    // Capability
    struct v4l2_capability cap;
    if (xioctl(m_FD, VIDIOC_QUERYCAP, &cap) < 0)
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
    if (xioctl(m_FD, VIDIOC_S_FMT, &fmt) < 0)
    {
        errno_exit("Format setting");
    }

    init_mmap();

    // StreamON
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_FD, VIDIOC_STREAMON, &type))
    {
        errno_exit("StreamON");
    }
    m_StreamOn = true;
    
    return S_OK;
}

void Device::Disconnect(){
    if (m_StreamOn)
    {
        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(m_FD, VIDIOC_STREAMOFF, &type);
        m_StreamOn = false;
    }

    if (m_Param.buffers)
    {
        for (int bufInd = 0; bufInd < N_IMAGES; bufInd++)
        {
            munmap(m_Param.buffers[bufInd].data, m_Param.buffers[bufInd].length);
        }
        free(m_Param.buffers);
        m_Param.buffers = nullptr;
    }

    close(m_FD);
}

int Device::GetDeviceInfo(std::vector<CInstanceId>* pInstanceIdList){
    return 1;
}

void* Device::GetFrameData(){
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_FD, &fds);

    struct timeval tv{};
    tv.tv_sec = 2;
    if (select(m_FD + 1, &fds, NULL, NULL, NULL) <= 0)
    {
        errno_exit("Frame capture timeout");
    }

    if(FD_ISSET(m_FD, &fds)){

        struct v4l2_buffer in_buf{};
        in_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        in_buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(m_FD, VIDIOC_DQBUF, &in_buf) < 0)
        {
            errno_exit("DQbuf");
        }

        if (xioctl(m_FD, VIDIOC_QBUF, &in_buf) < 0)
        {
            errno_exit("QBuf");
        }

        return m_Param.buffers[in_buf.index].data;
    }
}

int Device::GetData(float* data){
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_FD, &fds);

    struct timeval tv{};
    tv.tv_sec = 2;
    if (select(m_FD + 1, &fds, NULL, NULL, NULL) <= 0)
    {
        errno_exit("Frame capture timeout");
    }

    if(FD_ISSET(m_FD, &fds)){

        struct v4l2_buffer in_buf{};
        in_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        in_buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(m_FD, VIDIOC_DQBUF, &in_buf) < 0)
        {
            errno_exit("DQbuf");
        }
        
        memcpy((void*)data, m_Param.buffers[in_buf.index].data, m_Param.buffers[in_buf.index].length);

        if (xioctl(m_FD, VIDIOC_QBUF, &in_buf) < 0)
        {
            errno_exit("QBuf");
        }

        return 1;
    }
}

// Not usable yet
int Device::GetExposureT(){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE;
    if (xioctl(m_FD, VIDIOC_G_CTRL, &ctrl) < 0)
    {
        errno_exit("Get Exposure");
    }
    return ctrl.value;
}

// Not usable yet
void Device::SetExposureT(int value){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE;
    ctrl.value = value;
    if (xioctl(m_FD, VIDIOC_S_CTRL, &ctrl) < 0)
    {
        errno_exit("Set Exposure");
    }
}