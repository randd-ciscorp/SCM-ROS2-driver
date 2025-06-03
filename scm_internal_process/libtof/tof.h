#ifndef CIS_TOF_HEADER
#define CIS_TOF_HEADER

#include <vector>
#include <opencv2/opencv.hpp>

namespace cis
{
	struct CalibData
	{
		std::vector<float> cyclicLUT[2];
		cv::Mat gradient[2];

		float temperature_coef[2];
		float temperature_reftemp[2];

		cv::Mat LensLUT_x;
		cv::Mat LensLUT_y;
		cv::Mat LensLUT_z;
	};

	struct FusionData
	{
		cv::Mat R;
		cv::Mat t;
		cv::Mat cameraMatrix;
	};


	struct ToFParam
	{
		float MIN_REFLECTANCE;
		float MIN_CONFIDENCE;
		float KILL_FLYING_DELTA;
	};


	void init_calibdata(CalibData* calib, const std::string& calib_files_path);
	void init_fusiondata(FusionData* fusion);

	void tof_calc_distance_single(int freq, const char* src_ptr, char* dst_ptr, const ToFParam* tof_param, const CalibData* calib);
	void tof_calc_distance_dual(const char* src_ptr, char* dst_ptr, const ToFParam* tof_param, const CalibData* calib);
	void tof_calc_distance_hdr(const char* src_ptr, char* dst_ptr, const ToFParam* tof_param, const CalibData* calib);
	void rgbd_fusion(const char* tof_ptr, const char* rgb_ptr, char* dst_ptr, const ToFParam* tof_param, const CalibData* calib, const FusionData* fusion);


}

#endif /* CIS_TOF_HEADER */
