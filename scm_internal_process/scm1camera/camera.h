#ifndef CIS_CAMERA_HEADER
#define CIS_CAMERA_HEADER

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>

#include <array>
#include <algorithm>

#include <unistd.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <linux/usb/g_uvc.h>
#include <linux/usb/video.h>

#define CAMERA_BUFFER_NUM (3)
#define CAMERA_BUFFER_NUM_RAW (8)

inline int xioctl(int fd, uint req, void* arg)
{
	int r;
	do
	{
		r = ioctl(fd, req, arg);
	} while(r == -1 && errno == EINTR);
	return r;
}

namespace cis
{
	struct CameraBuffer
	{
		struct v4l2_buffer v4l2buf;
		void* data;
		size_t size;
	};

	class CameraEvent
	{
	public:
		CameraEvent() : max_fd(0)
		{
			FD_ZERO(&rfds);
			FD_ZERO(&wfds);
			FD_ZERO(&efds);
		}

		int wait(bool streaming)
		{
			FD_ZERO(&r);
			FD_ZERO(&w);
			FD_ZERO(&e);
			r = rfds;
			w = wfds;
			e = efds;

			if(streaming)
			{
				return select(max_fd + 1, &r, &w, &e, nullptr);
			}
			else
			{
				return select(max_fd + 1, nullptr, nullptr, &e, nullptr);
			}
		}

		bool can_read(int fd)
		{
			return FD_ISSET(fd, &r);
		}

		bool can_write(int fd)
		{
			return FD_ISSET(fd, &w);
		}

		bool event_happen(int fd)
		{
			return FD_ISSET(fd, &e);
		}

		fd_set r, w, e;
		fd_set rfds;
		fd_set wfds;
		fd_set efds;
		int max_fd;
	};


	class CameraBaseColor
	{
	public:
		CameraBaseColor() = delete;
		CameraBaseColor(size_t width, size_t height, int pixelformat)
			: width(width), height(height), pixelformat(pixelformat) { }

		int open();
		void close();
		int fd() const;

		void init(CameraEvent* event);

		int connect();
		int disconnect();

		int stream_on();
		int stream_off();

        int get_width() const {return width;}
        int get_height() const {return height;}

		CameraBuffer* pop();
		void push(CameraBuffer* buffer);

		int get(int req, int intf, int param_index);
		void set(int req, int intf, int param_index, int value);


	protected:
		bool is_streaming;

		const size_t width;
		const size_t height;
		const int pixelformat;
		struct v4l2_format fmt;

		CameraBuffer buffer[CAMERA_BUFFER_NUM ];
		int dma_fd[CAMERA_BUFFER_NUM];

		int _fd;
		int _cma_fd;
		int _dev_num;

		struct CameraParameter
		{
			int cur_val;
			int min_val;
			int max_val;
			int step;
			int def_val;
			int info;
		};

		CameraParameter input[0x12];
		CameraParameter processing[0x13];

		void init_param();
	};


	class CameraBaseRAW
	{
	public:
		CameraBaseRAW() = delete;
		CameraBaseRAW(size_t width, size_t height, int pixelformat)
		       	: width(width), height(height), pixelformat(pixelformat) { }


		int open();
		void close();
		int fd() const;

		void init(CameraEvent* event);

		int connect();
		int disconnect();

		int stream_on();
		int stream_off();

		CameraBuffer* pop();
		void push(CameraBuffer* buffer);

        int get_width() const {return width;}
        int get_height() const {return height;}

	protected:
		bool is_streaming;

		const size_t width;
		const size_t height;
		const int pixelformat;
		struct v4l2_format fmt;

		CameraBuffer buffer[CAMERA_BUFFER_NUM_RAW];

		int _fd;
	};


	class CameraOutput
	{
	public:
		int open();
		void close();
		int fd() const;

		void init(size_t width, size_t height, int pixelformat, CameraEvent* event);

		int connect();
		int disconnect();

		int stream_on();
		int stream_off();

		CameraBuffer* pop();
		void push(CameraBuffer* buffer);

	private:
		bool is_streaming;

		size_t _width;
		size_t _height;
		int _pixelformat;
		struct v4l2_format fmt;

		int _fd;
		CameraBuffer buffer[CAMERA_BUFFER_NUM];
	};


	class CameraAR0234 : public CameraBaseColor
	{
	public:
		CameraAR0234() : CameraBaseColor(1920U, 1080U, V4L2_PIX_FMT_YUYV) { }
	};

	class CameraAR0234RAW : public CameraBaseRAW
	{
	public:
		CameraAR0234RAW() : CameraBaseRAW(1920U, 1200U, V4L2_PIX_FMT_Y10) { }
	};

	class CameraIMX715 : public CameraBaseColor
	{
	public:
		CameraIMX715() : CameraBaseColor(3840U, 2160U, V4L2_PIX_FMT_YUYV) { }
	};

	class CameraIMX715RAW : public CameraBaseRAW
	{
	public:
		CameraIMX715RAW() : CameraBaseRAW(3840U, 2160U, V4L2_PIX_FMT_Y10) { }
	};

	class CameraIMX570 : public CameraBaseRAW
	{
	public:
		CameraIMX570() : CameraBaseRAW(1280U, 3840U, V4L2_PIX_FMT_SBGGR12) { }
	};

	class CameraGenX320Histo : public CameraBaseRAW
	{
	public:
        CameraGenX320Histo() : CameraBaseRAW(320U, 320U, V4L2_PIX_FMT_YUV32) { }
	};

	class CameraGenX320EVT : public CameraBaseRAW
	{
	public:
        CameraGenX320EVT() : CameraBaseRAW(4096U, 320U, V4L2_PIX_FMT_SBGGR8) { }
	};

} /* namespace cis */

#endif /* CIS_CAMERA_HEADER */
