#include "camera.h"

namespace cis
{
	int CameraOutput::open()
	{
		is_streaming = false;
		
		for(int i = 3; i < 10; ++i)
		{
			char dev_name[32];
			sprintf(dev_name, "/dev/video%d", i);

			int fd = ::open(dev_name, O_RDWR);
			if(fd < 0)
			{
				continue;
			}
			
			struct v4l2_capability cap{};
			xioctl(fd, VIDIOC_QUERYCAP, &cap);
			
			if(strcmp((const char*)cap.driver, "uvc_gadget") == 0)
			{
				_fd = fd;
				break;
			}
			::close(fd);
		}

		return 0;
	}

	void CameraOutput::close()
	{
		::close(_fd);
	}

	int CameraOutput::fd() const
	{
		return _fd;
	}


	void CameraOutput::init(size_t width, size_t height, int pixelformat, CameraEvent* event)
	{
		_width = width;
		_height = height;
		_pixelformat = _pixelformat;
	
		/* set event type */
		struct v4l2_event_subscription sub{};
		sub.type = UVC_EVENT_SETUP;
		xioctl(_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
		sub.type = UVC_EVENT_DATA;
		xioctl(_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
		sub.type = UVC_EVENT_STREAMON;
		xioctl(_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
		sub.type = UVC_EVENT_STREAMOFF;
		xioctl(_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
		sub.type = UVC_EVENT_CONNECT;
		xioctl(_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
		sub.type = UVC_EVENT_DISCONNECT;
		xioctl(_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
		
		/* event watch */
		FD_SET(_fd, &event->wfds);
		FD_SET(_fd, &event->efds);
		event->max_fd = std::max(_fd, event->max_fd);
	}

	int CameraOutput::connect()
	{
		int ret = 0;

		fmt = {};
		fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		xioctl(_fd, VIDIOC_G_FMT, &fmt);

		return ret;
	}

	int CameraOutput::stream_on()
	{
		if(is_streaming)
		{
			return 0;
		}
		
		int ret = 0;

		/* request buffers */
		struct v4l2_requestbuffers reqbufs{};
		reqbufs.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		reqbufs.count = CAMERA_BUFFER_NUM;
		reqbufs.memory = V4L2_MEMORY_MMAP;
		if((ret = xioctl(_fd, VIDIOC_REQBUFS, &reqbufs)) < 0)
		{
			perror("VIDIOC_REQBUFS");
			return ret;
		}

		/* prepare buffers */
		for(int i = 0; i < CAMERA_BUFFER_NUM; ++i)
		{
			struct v4l2_buffer v4l2buf{};
			v4l2buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			v4l2buf.memory = V4L2_MEMORY_MMAP;
			v4l2buf.index = i;
			v4l2buf.bytesused = fmt.fmt.pix.sizeimage;
			if((ret = xioctl(_fd, VIDIOC_QUERYBUF, &v4l2buf)) < 0)
			{
				perror("VIDIOC_QUERYBUF");
				return ret;
			}

			buffer[i].v4l2buf = v4l2buf;
			buffer[i].size = v4l2buf.length;
			buffer[i].data = mmap(nullptr, v4l2buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
					_fd, v4l2buf.m.offset);
			if((ret = xioctl(_fd, VIDIOC_QBUF, &v4l2buf)) < 0)
			{
				perror("VIDIOC_QBUF");
				return ret;
			}
		}

		int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		ret = xioctl(_fd, VIDIOC_STREAMON, &type);

		if(ret == 0)
		{
			is_streaming = true;
		}

		return ret;
	}

	int CameraOutput::stream_off()
	{
		if(is_streaming)
		{
			int ret = 0;
			int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			ret = xioctl(_fd, VIDIOC_STREAMOFF, &type);
			if(ret == 0)
			{
				is_streaming = false;
			}

			for(int i = 0; i < CAMERA_BUFFER_NUM; ++i)
			{
				munmap(buffer[i].data, buffer[i].size);
			}
			return ret;
		}
		else
		{
			return 0;
		}
	}

	int CameraOutput::disconnect()
	{
		return stream_off();
	}

	CameraBuffer* CameraOutput::pop()
	{
		struct v4l2_buffer v4l2buf{};
		v4l2buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		v4l2buf.memory = V4L2_MEMORY_MMAP;

		if(xioctl(_fd, VIDIOC_DQBUF, &v4l2buf) < 0)
		{
			perror("VIDIOC_DQBUF");
		}
		
		buffer[v4l2buf.index].v4l2buf = v4l2buf;
		return &buffer[v4l2buf.index];
	}

	void CameraOutput::push(CameraBuffer* inbuf)
	{
		xioctl(_fd, VIDIOC_QBUF, &(*inbuf).v4l2buf);
	}

} /* namespace cis */
