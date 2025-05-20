#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <sys/ioctl.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <string>

#define N_CAP_BUF	4

// Error type
#define S_OK			0
#define E_POINTER		EFAULT
#define E_FAIL			ENODATA
#define E_INVALIDARG	EINVAL
#define E_OUTOFMEMORY	ENOMEM
#define E_HANDLE		ENODATA

inline int xioctl(int fd, uint req, void* arg){
    int r;
	do {
		r = ioctl(fd, req, arg);
	} while(r == -1 && errno == EINTR);
	return r;
}

struct RequestBuffer
{
    void *data;
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
public:
    Device();
    ~Device();

    int connect(int height, int width);
    void disconnect();

    DevInfo getInfo() const;
	int getData(uint8_t* data);
    bool isConnected() const;

private:
    DevInfo devInfo_;

    int	fd_ = -1;

	u_int8_t *data_;
    unsigned int bufInd_;
	struct RequestBuffer* buffers_;

    bool isStreamOn_ = false;
	
    void initMmap();

	int errnoExit(const char *s) const;
};

#endif