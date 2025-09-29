// Copyright 2025 CIS Corporation
#ifndef CIS_SCM__DEVICE_H_
#define CIS_SCM__DEVICE_H_

#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#define N_CAP_BUF 4

// Error type
#define S_OK 0
#define E_POINTER EFAULT
#define E_FAIL ENODATA
#define E_INVALIDARG EINVAL
#define E_OUTOFMEMORY ENOMEM
#define E_HANDLE ENODATA

namespace cis_scm
{

inline int xioctl(int fd, u_int64_t req, void * arg)
{
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

inline int errnoExit(const char * s)
{
    fprintf(stderr, "'%s': '%d, %s \n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
    return errno;
}

inline int errnoPrint(const char * s)
{
    fprintf(stderr, "'%s': '%d, %s \n", s, errno, strerror(errno));
    return errno;
}

struct RequestBuffer
{
    void * data;
    size_t length;
};

struct DevInfo
{
    std::string devName;
    std::string driverVers;
    std::string sn;

    int width;
    int height;
};

class Device
{
  protected:
    Device();
    ~Device();

  public:
    virtual DevInfo getInfo() const = 0;
    virtual int getData(uint8_t * data) = 0;
    bool isConnected() const;

  protected:
    DevInfo devInfo_;

    int fd_ = -1;
    unsigned int bufInd_;
    struct RequestBuffer * buffers_;

    bool isStreamOn_ = false;
};
}  // namespace cis_scm
#endif  // CIS_SCM__DEVICE_H_
