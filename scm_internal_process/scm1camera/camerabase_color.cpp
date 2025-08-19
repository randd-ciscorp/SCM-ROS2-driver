#include "camera.h"

namespace cis
{
	int CameraBaseColor::open()
	{
		is_streaming = false;
		_cma_fd = ::open("/dev/dma_heap/linux,cma", O_RDWR);
		
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

			if(strcmp((const char*)cap.driver, "viv_v4l2_device") == 0)
			{
				_fd = fd;
				_dev_num = i;
				break;
			}

			::close(fd);
		}

		return 0;
	}

	void CameraBaseColor::close()
	{
		::close(_cma_fd);
		::close(_fd);
	}

	int CameraBaseColor::fd() const
	{
		return _fd;
	}


	void CameraBaseColor::init(CameraEvent* event)
	{
		/* event watch */
		FD_SET(_fd, &event->rfds);
		event->max_fd = std::max(_fd, event->max_fd);	
		
		init_param();
	}


	int CameraBaseColor::connect()
	{
		int ret = 0;
		
		/* get bytesperline */
		
		fmt = {};
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width = width;
		fmt.fmt.pix.height = height;
		fmt.fmt.pix.pixelformat = pixelformat;
		fmt.fmt.pix.field = V4L2_FIELD_ANY;
		if((ret = xioctl(_fd, VIDIOC_S_FMT, &fmt)) < 0)
		{
			perror("VIDIOC_S_FMT");
			return ret;
		}
		xioctl(_fd, VIDIOC_G_FMT, &fmt);

		/* alloc dmabuf */
		for(int i = 0; i < CAMERA_BUFFER_NUM; ++i)
		{
			struct dma_heap_allocation_data dma_alloc_data{};
			dma_alloc_data.len = fmt.fmt.pix.sizeimage;
			dma_alloc_data.fd_flags = O_RDWR | O_CLOEXEC;
			if((ret = xioctl(_cma_fd, DMA_HEAP_IOCTL_ALLOC, &dma_alloc_data)) < 0)
			{
				perror("DMA_HEAP_IOCTL_ALLOC");
				return ret;
			}
			dma_fd[i] = dma_alloc_data.fd;
		}
		
		return ret;
	}


	int CameraBaseColor::stream_on()
	{
		if(is_streaming)
		{
			return 0;
		}

		int ret = 0;


		/* request buffers */
		struct v4l2_requestbuffers reqbufs{};
		reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		reqbufs.count = CAMERA_BUFFER_NUM;
		reqbufs.memory = V4L2_MEMORY_DMABUF;
		if((ret = xioctl(_fd, VIDIOC_REQBUFS, &reqbufs)) < 0)
		{
			perror("VIDIOC_REQBUFS");
			return ret;
		}


		/* prepare buffer */
		for(int i = 0; i < CAMERA_BUFFER_NUM; ++i)
		{
			struct v4l2_buffer v4l2buf{};
			v4l2buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			v4l2buf.memory = V4L2_MEMORY_DMABUF;
			v4l2buf.index = i;
			if((ret = xioctl(_fd, VIDIOC_QUERYBUF, &v4l2buf)) < 0)
			{
				perror("VIDIOC_QUERYBUF");
				return ret;
			}	

			buffer[i].size = v4l2buf.length;

			buffer[i].data = mmap(nullptr, v4l2buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, 
					dma_fd[i], v4l2buf.m.offset);
			buffer[i].v4l2buf = v4l2buf;
			v4l2buf.m.fd = dma_fd[i];
			if((ret = xioctl(_fd, VIDIOC_QBUF, &v4l2buf)) < 0)
			{
				perror("VIDIOC_QBUF");
				return ret;
			}
		}

		int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		ret = xioctl(_fd, VIDIOC_STREAMON, &type);
		if(ret == 0)
		{
			is_streaming = true;
		}

		return ret;
	}
	
	int CameraBaseColor::stream_off()
	{
		if(is_streaming)
		{
			int ret = 0;
			int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			ret = xioctl(_fd, VIDIOC_STREAMOFF, &type);
			if(ret == 0)
			{
				is_streaming = false;
			}

			for(int i = 0; i < CAMERA_BUFFER_NUM; ++i)
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


	int CameraBaseColor::disconnect()
	{
		int ret = 0;
		ret = stream_off();
		
		for(int i = 0; i < CAMERA_BUFFER_NUM; ++i)
		{
			::close(dma_fd[i]);
			dma_fd[i] = 0;
		}
		return ret;
	}


	CameraBuffer* CameraBaseColor::pop()
	{
		struct v4l2_buffer v4l2buf{};
		v4l2buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v4l2buf.memory = V4L2_MEMORY_DMABUF;

		xioctl(_fd, VIDIOC_DQBUF, &v4l2buf);
		
		buffer[v4l2buf.index].v4l2buf = v4l2buf;
		return &buffer[v4l2buf.index];
	}

	void CameraBaseColor::push(CameraBuffer* in_buf)
	{
		xioctl(_fd, VIDIOC_QBUF, &in_buf->v4l2buf);
	}

	void CameraBaseColor::init_param()
	{
		memset(input, 0, sizeof(CameraParameter) * 18);
		memset(processing, 0, sizeof(CameraParameter) * 16);

		auto set_h = [](CameraParameter& param, int _def, int _min, int _max, int _step, int _info)
		{
			param.def_val = param.cur_val = _def;
			param.min_val = _min;
			param.max_val = _max;
			param.step = _step;
			param.info = _info;
		};

		set_h(processing[UVC_PU_BRIGHTNESS_CONTROL], -30, -128, 127, 1, 0x0f); 		//brightness
		set_h(processing[UVC_PU_CONTRAST_CONTROL], 100, 0, 199, 1, 0x03); 		//contrast
		set_h(processing[UVC_PU_HUE_CONTROL], 0, -90, 89, 1, 0x03); 			//hue
		set_h(processing[UVC_PU_SATURATION_CONTROL], 100, 0, 199, 1, 0x03); 		//saturation
	}

	int CameraBaseColor::get(int req, int intf, int param_index)
	{
		CameraParameter* param = intf == 1 ? input : processing;

		int ret = 0;
		switch(req)
		{
		case UVC_GET_CUR:
			ret = param[param_index].cur_val;
			break;
		case UVC_GET_MIN:
			ret = param[param_index].min_val;
			break;
		case UVC_GET_MAX:
			ret = param[param_index].max_val;
			break;
		case UVC_GET_RES:
			ret = param[param_index].step;
			break;
		case UVC_GET_DEF:
			ret = param[param_index].def_val;
			break;
		case UVC_GET_INFO:
			ret = param[param_index].info;
			break;
		}

		return ret;
	}
	
	void CameraBaseColor::set(int req, int intf, int param_index, int val)
	{
		int ret;
		char vvext[256];
		
		CameraParameter* param = intf == 1 ? input : processing;
		bool changed = param[param_index].cur_val != std::clamp(val, param[param_index].min_val, param[param_index].max_val);
		param[param_index].cur_val = std::clamp(val, param[param_index].min_val, param[param_index].max_val);

		if(changed)
		{	
			sprintf(vvext, "/opt/imx8-isp/bin/vvext %d 'CPROC ON/OFF' '%d'", _dev_num, 1);
			ret = system(vvext);
		}

		if(intf == 1)
		{

		}
		else if(intf == 2)
		{
		
			switch(param_index)
			{
			case UVC_PU_BRIGHTNESS_CONTROL:
				if(val == -128)
				{
					sprintf(vvext, "/opt/imx8-isp/bin/vvext %d 'CPROC ON/OFF' '%d'", _dev_num, 0);
				}
				else
				{
					sprintf(vvext, "/opt/imx8-isp/bin/vvext %d 'Adjust brightness' '%d'",_dev_num, val);
				}
				break;

			case UVC_PU_CONTRAST_CONTROL:
				sprintf(vvext, "/opt/imx8-isp/bin/vvext %d 'Adjust contrast' '%f'",_dev_num, val / 100.f);
				break;

			case UVC_PU_HUE_CONTROL:
				sprintf(vvext, "/opt/imx8-isp/bin/vvext %d 'Adjust HUE' '%d'",_dev_num, val);
				break;

			case UVC_PU_SATURATION_CONTROL:
				sprintf(vvext, "/opt/imx8-isp/bin/vvext %d 'Adjust saturation' '%f'",_dev_num, val / 100.f);
				break;
			}

			ret = system(vvext);
			if(ret < 0)
			{
				printf("vvext error\n");
			}
		}
		else
		{

		}
	}
}
