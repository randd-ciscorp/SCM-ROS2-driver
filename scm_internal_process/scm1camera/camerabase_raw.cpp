#include "camera.h"

namespace cis
{
	int CameraBaseRAW::open()
	{
		is_streaming = false;

		for(int i = 2; i < 10; ++i)
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

			if(strcmp((const char*)cap.driver, "mxc-isi-cap_v1") == 0)
			{
				_fd = fd;
				break;
			}

			::close(fd);
		}

		return 0;
	}

	void CameraBaseRAW::close()
	{
		::close(_fd);
	}

	int CameraBaseRAW::fd() const
	{
		return _fd;
	}

	void CameraBaseRAW::init(CameraEvent* event)
	{
		FD_SET(_fd, &event->rfds);
		event->max_fd = std::max(_fd, event->max_fd);
	}

	int CameraBaseRAW::connect()
	{
		int ret = 0;

		/* set_format */
		fmt = {};
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		fmt.fmt.pix_mp.width = width;
		fmt.fmt.pix_mp.height = height;
		fmt.fmt.pix_mp.num_planes = 1;
		fmt.fmt.pix_mp.pixelformat = pixelformat;
		fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
		if((ret = xioctl(_fd, VIDIOC_S_FMT, &fmt)) < 0)
		{
			perror("VIDIOC_S_FMT");
			return ret;
		}
		xioctl(_fd, VIDIOC_G_FMT, &fmt);	/* fill empty fields */

		return ret;
	}

	int CameraBaseRAW::stream_on()
	{
		if(is_streaming)
		{
			return 0;
		}

		int ret = 0;

		/* request buffers */
		struct v4l2_requestbuffers reqbufs{};
		reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		reqbufs.count = CAMERA_BUFFER_NUM_RAW;
		reqbufs.memory = V4L2_MEMORY_MMAP;
		if((ret = xioctl(_fd, VIDIOC_REQBUFS, &reqbufs)) < 0)
		{
			perror("VIDIOC_REQBUFS");
			return ret;
		}

		/* init buffers */
		for(int i = 0; i < CAMERA_BUFFER_NUM_RAW; ++i)
		{
			struct v4l2_buffer v4l2buf{};
			struct v4l2_plane v4l2plane[1]{};
			v4l2buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			v4l2buf.memory = V4L2_MEMORY_MMAP;
			v4l2buf.index = i;
			v4l2buf.m.planes = v4l2plane;
			v4l2buf.length = 1;
			if((ret = xioctl(_fd, VIDIOC_QUERYBUF, &v4l2buf)) < 0)
			{
				perror("VIDIOC_QUERYBUF");
				return ret;
			}

			buffer[i].size = v4l2buf.m.planes[0].length;
			buffer[i].data = mmap(nullptr, v4l2buf.m.planes[0].length, PROT_READ | PROT_WRITE,
					MAP_SHARED, _fd, v4l2buf.m.planes[0].m.mem_offset);
			if((ret = xioctl(_fd, VIDIOC_QBUF, &v4l2buf)) < 0)
			{
				perror("VIDIOC_QBUF");
				return ret;
			}
		}

		int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		ret = xioctl(_fd, VIDIOC_STREAMON, &type);
		if(ret == 0)
		{
			is_streaming = true;
		}

		return ret;
	}

	int CameraBaseRAW::stream_off()
	{
		if(is_streaming)
		{
			int ret = 0;
			int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			ret = xioctl(_fd, VIDIOC_STREAMOFF, &type);
			if(ret == 0)
			{
				is_streaming = false;
			}

			for(int i = 0; i < CAMERA_BUFFER_NUM_RAW; ++i)
			{
				munmap(buffer[i].data, buffer[i].size);
			}

			return 0;
		}
		else
		{
			return 0;
		}
	}

	int CameraBaseRAW::disconnect()
	{
		return stream_off();
	}


	CameraBuffer* CameraBaseRAW::pop()
	{
		struct v4l2_buffer v4l2buf{};
		struct v4l2_plane v4l2plane[1]{};
		v4l2buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		v4l2buf.memory = V4L2_MEMORY_MMAP;
		v4l2buf.m.planes = v4l2plane;
		v4l2buf.length = 1;

		xioctl(_fd, VIDIOC_DQBUF, &v4l2buf);
		buffer[v4l2buf.index].v4l2buf = v4l2buf;
		return &buffer[v4l2buf.index];
	}

	void CameraBaseRAW::push(CameraBuffer* inbuf)
	{
		xioctl(_fd, VIDIOC_QBUF, &inbuf->v4l2buf);
	}

} /* namespace cis */
