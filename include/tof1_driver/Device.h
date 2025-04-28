#include <pthread.h>
#include <semaphore.h>
#include <string>
#include <vector>

#define N_IMAGES	3

// Error type
#define S_OK			0
#define E_POINTER		EFAULT
#define E_FAIL			ENODATA
#define E_INVALIDARG	EINVAL
#define E_OUTOFMEMORY	ENOMEM
#define E_HANDLE		ENODATA

typedef long	LONG;
typedef unsigned char	BYTE;

struct RequestBuffer
{
    void *data;
    size_t length;
};

// スレッドに渡すパラメータ構造体
struct ThreadParam
{
	pthread_t	thread_id;
	bool	run;
	bool	received;
	int		fd;
	int		data_len;
	sem_t	*fd_sem;		// Semaphore
	sem_t	*recv_sem;		// Semaphore
	BYTE	*data;
    unsigned int bufInd;
	struct RequestBuffer* buffers;
};

class CInstanceId
{
public:
	std::string VendorID;
	std::string ProductID;
	std::string SerialNo;
	std::string DeviceName;
	std::string Manufacturer;
	std::string Product;
};

class Device
{ 
public:
    Device(/* args */);
    ~Device();

    int Connect(const char* serialNum, int height, int width);
    void Disconnect();

    // 取得／設定
	int GetDeviceCount();
	int GetDeviceInfo(std::vector<CInstanceId>* pInstanceIdList);
	int GetData(float* data);
    void* GetFrameData();
    bool IsConnected();

private:
	int FD = -1;

    bool streamOn_ = false;
	// thread
	struct ThreadParam thrdParams_;
	
    void init_mmap();

	//int CreateInstanceIdList();
	int errno_exit(const char *s);
	//static int xioctl(int fd, int request, void *arg, sem_t *fd_sem);
	static int xioctl(int fd, int request, void *arg);

};


