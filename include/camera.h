#ifndef DRIVER_CAMERA_H
#define DRIVER_CAMERA_H


#include <iostream>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

namespace dreame
{
	
enum cc_cap_prop
{
	CC_EXPOSURE_ABSOLUTE = 0,
	CC_FRAME_FPS = 1
};
	
class V4L2Capture
{
public:
	V4L2Capture(char *dev_name);
	int openDevice();
	int initDevice(int height, int width, int color=cv::IMREAD_GRAYSCALE);
	int initBuffers(unsigned int buffer_size=2);
	int startCapture();
	void release();
	
	void set(int option, double value);
	
	int operator>>(cv::Mat& image);
	
	/**
	@brief 列出摄像头支持的像素格式
	*/
	void getDevSupportedFormat();
	
	double getFPS();
	int closeDevice();
	
private:
	int qbuf();
	
	int stopCapture();
	int freeBuffers();
	
	
private:
	const char *dev_name;
	int fd_cam;
	
	int width, height, color;
	
	struct v4l2_capability cam_cap;
	struct v4l2_fmtdesc cam_fmtdesc;
	struct v4l2_cropcap cam_cropcap;
	struct v4l2_crop cam_crop;
	struct v4l2_format cam_format;
	struct v4l2_control cam_ctrl;
	struct v4l2_streamparm cam_streamparm;
	
	struct cam_buffer
	{
		void *start;
		size_t length;
	};
	cam_buffer *buffers;
	unsigned int n_buffers;
	
	uchar *pFrame;
	size_t frame_size;
	int frame_index;

};

}

#endif
