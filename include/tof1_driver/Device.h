#include <sys/ioctl.h>

#include <string>
#include <vector>

#define N_CAP_BUF	3

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

class Device
{ 
public:
    Device();
    ~Device();

    int connect(const char* serialNum, int height, int width);
    void disconnect();

	int getData(float* data);
    void* getFrameData();
    bool isConnected();

private:
    int	fd_ = -1;

	u_int8_t *data_;
    unsigned int bufInd_;
	struct RequestBuffer* buffers_;

    bool isStreamOn_ = false;
	
    void initMmap();

	int errnoExit(const char *s);
};
