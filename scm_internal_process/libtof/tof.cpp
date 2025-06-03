#include <cstdint>
#include <numbers>
#include <arm_neon.h>
#include <opencv2/opencv.hpp>

#include "tof.h"
#include "tof_math.h"

namespace cis
{
	struct RAW
	{
		uint16_t A;
		uint16_t B;
	};

	struct RGBD
	{
		float x;
		float y;
		float z;
		uint8_t b;
		uint8_t g;
		uint8_t r;
		uint8_t a;
	};

	template <int freq>
	void __tof_calc_distance_single(const char* src, char* dst, const ToFParam* tof_param, const CalibData* calib)
	{
		constexpr int width = 640;
		constexpr int height = 480;
		constexpr int phase_num = 4;
		constexpr int freq_num = 1;

		constexpr float pi = std::numbers::pi_v<float>;
		constexpr float c0 = 299.792458f;
		constexpr float inv_2pi = 1.f / (2.f * pi);

		constexpr int freq_list[freq_num] = { freq };
		constexpr float maxD[freq_num] = { c0 / (2.f * freq) };

		cv::Mat confidence(height, width, CV_16SC1);
		cv::Mat blur_confidence(height, width, CV_16SC1);

		cv::Mat radialDepth(height, width, CV_32FC1);
		static cv::Mat radialTmp = cv::Mat::zeros(height, width, CV_32FC1);
		static cv::Mat radialMixRate(height, width, CV_32FC1);

		RAW* raw = (RAW*)src;
		RAW* Q[freq_num][phase_num] = 
		{
			{ 
				&raw[width * height * 0],
				&raw[width * height * 1],
				&raw[width * height * 2],
				&raw[width * height * 3],	
			},
		};

		cv::Mat Q13[freq_num] = 
		{
			cv::Mat(height, width, CV_16SC1),
		};
		cv::Mat Q20[freq_num] = 
		{
			cv::Mat(height, width, CV_16SC1),
		};

		cv::Mat ret(height, width, CV_32FC3, dst);

		/* CALC DIFF Q */
		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=8)
			{
				int pos = x + y * width;

				int16x8_t vconf = vdupq_n_s16(0);

				for(int f = 0; f < freq_num; ++f)
				{
					int16x8_t vq[phase_num];
					for(int p = 0; p < phase_num; ++p)
					{
						int16x8x2_t vr = vld2q_s16((int16_t*)&Q[f][p][pos]);
						int16x8_t vqa = vr.val[0];
						int16x8_t vqb = vr.val[1];
						vq[p] = vsubq_s16(vqa, vqb);
					}

					int16x8_t vq13 = vsubq_s16(vq[1], vq[3]);
					int16x8_t vq20 = vsubq_s16(vq[2], vq[0]);

					vconf = vaddq_s16(vconf, vaddq_s16(vabsq_s16(vq13), vabsq_s16(vq20)));

					vst1q_s16(Q13[f].ptr<int16_t>() + pos, vq13);
					vst1q_s16(Q20[f].ptr<int16_t>() + pos, vq20);
				}

				vst1q_s16(confidence.ptr<int16_t>() + pos, vconf);
			}
		}

		/* REFINEMENT RAW */
		#pragma omp parallel for schedule(guided)	
		for(int i = 0; i < 4; ++i)
		{
			for(int f = 0; f < freq_num; ++f)
			{
				cv::Rect roi(0, height / 4 * i, width, height / 4);
				cv::blur(Q13[f](roi), Q13[f](roi), { 3, 3 });
				cv::blur(Q20[f](roi), Q20[f](roi), { 3, 3 });
			}
		}

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos = x + y * width;

				float32x4_t new_phase[freq_num];

				/* CALC PHASE */
				for(int f = 0; f < freq_num; ++f)
				{
					constexpr int c = freq == 20 ? 0 : 1;
					int16x4_t vq13_s16 = vld1_s16(Q13[f].ptr<int16_t>() + pos);
					int16x4_t vq20_s16 = vld1_s16(Q20[f].ptr<int16_t>() + pos);

					float32x4_t vq13_f32 = vcvtq_f32_s32(vmovl_s16(vq13_s16));
					float32x4_t vq20_f32 = vcvtq_f32_s32(vmovl_s16(vq20_s16));

					float32x4_t raw_phase = vaddq_f32(vatan2q_f32(vq13_f32, vq20_f32), vdupq_n_f32(pi));
					int32x4_t scaled_raw_phase = vcvtq_s32_f32(vmulq_n_f32(raw_phase, 1023.f * inv_2pi));
					float32x4_t cyc_phase;
					for(int i = 0; i < 4; ++i)
					{
						cyc_phase[i] = calib->cyclicLUT[c][scaled_raw_phase[i]];
					}

					float32x4_t gdc_offset = vld1q_f32(calib->gradient[c].ptr<float>() + pos);

					float32x4_t tec_offset = 
						vdupq_n_f32(0.f);
						// temp_offset[i];

					new_phase[f] = vaddq_f32(cyc_phase, vaddq_f32(gdc_offset, tec_offset));
					new_phase[f] = fmodf_2pi(vaddq_f32(new_phase[f], vdupq_n_f32(2.f * pi))); 

				}

				float32x4_t dealiased = vmulq_n_f32(new_phase[0], inv_2pi * maxD[0]);

				vst1q_f32(radialDepth.ptr<float>() + pos, dealiased);
			}
		}



		/* TEMPORAL ANTIALIAS */
		cv::copyMakeBorder(radialDepth, radialDepth, 1, 1, 1, 1, cv::BORDER_DEFAULT);

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos0 = x + y * width;

				float32x4_t anti_aliased = vld1q_f32(radialTmp.ptr<float>() + pos0);
				float32x4_t mix_rate = vminq_f32(vld1q_f32(radialMixRate.ptr<float>() + pos0), vdupq_n_f32(0.5f));
				float32x4_t in[3][3];
				// more optimize !
				for(int i = 0; i < 3; ++i)
				{
					for(int j = 0; j < 3; ++j)
					{
						int pos1 = (x + j) + (y + i) * (width + 2);
						in[i][j] = vld1q_f32(radialDepth.ptr<float>() + pos1);
					}
				}

				float32x4_t alpha = vsubq_f32(vdupq_n_f32(1.f), mix_rate);
				float32x4_t beta = mix_rate;

				anti_aliased = vaddq_f32(vmulq_f32(vmulq_f32(anti_aliased, anti_aliased), alpha),
							 vmulq_f32(vmulq_f32(in[1][1], in[1][1]), beta));
				anti_aliased = vsqrtq_f32(anti_aliased);

				float32x4_t min_depth = vminq_f32(vminq_f32(vminq_f32(in[1][1], in[0][0]), vminq_f32(in[0][1], in[0][2])), in[1][0]);
				float32x4_t max_depth = vmaxq_f32(vmaxq_f32(vmaxq_f32(in[1][1], in[0][0]), vmaxq_f32(in[0][1], in[0][2])), in[1][0]);
				alpha = vdupq_n_f32(0.5f);
				beta = vdupq_n_f32(0.5f);

				min_depth = vaddq_f32(vmulq_f32(min_depth, alpha), 
						      vmulq_f32(vminq_f32(vminq_f32(vminq_f32(in[1][2], in[2][0]), vminq_f32(in[2][1], in[2][2])), min_depth), beta));

				max_depth = vaddq_f32(vmulq_f32(max_depth, alpha), 
						      vmulq_f32(vmaxq_f32(vmaxq_f32(vmaxq_f32(in[1][2], in[2][0]), vmaxq_f32(in[2][1], in[2][2])), max_depth), beta));

				float32x4_t pre_clamping = anti_aliased;
				anti_aliased = vminq_f32(vmaxq_f32(anti_aliased, min_depth), max_depth);

				mix_rate = vdivq_f32(vdupq_n_f32(1.f), vaddq_f32(vdupq_n_f32(1.f), vdivq_f32(vdupq_n_f32(1.f), mix_rate)));

				float32x4_t diff = vsubq_f32(anti_aliased, pre_clamping);
				float32x4_t clamp_amount = vmulq_f32(diff, diff);

				mix_rate = vfmaq_f32(mix_rate, clamp_amount, vdupq_n_f32(4.f));
				mix_rate = vminq_f32(vmaxq_f32(mix_rate, vdupq_n_f32(0.05f)), vdupq_n_f32(0.5f));

				vst1q_f32(radialTmp.ptr<float>() + pos0, anti_aliased);
				vst1q_f32(radialMixRate.ptr<float>() + pos0, mix_rate);
			}
		}

		blur_confidence = cv::abs(Q13[0]) + cv::abs(Q20[0]);


		/* POST FILTER */
		cv::copyMakeBorder(radialTmp, radialDepth, 1, 1, 1, 1, cv::BORDER_DEFAULT);

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos0 = x + y * width;
				float32x4_t in[3][3];
				for(int i = 0; i < 3; ++i)
				{
					for(int j = 0; j < 3; ++j)
					{
						int pos1 = (x + j) + (y + i) * (width + 2);
						in[i][j] = vld1q_f32(radialDepth.ptr<float>() + pos1);
					}
				}
				
				/* CONFIDENCE FILTER*/
				float32x4_t vconf = vcvtq_f32_s32(vmovl_s16(vld1_s16(blur_confidence.ptr<int16_t>() + pos0)));
				float32x4_t vdepth = in[1][1];
				float32x4_t vinv_d2 = vdivq_f32(vdupq_n_f32(1.f), vmulq_f32(vdepth, vdepth));

				uint32x4_t cond_ref = vcltq_f32(vconf, vmulq_f32(vdupq_n_f32(tof_param->MIN_REFLECTANCE), vinv_d2));
				uint32x4_t cond_conf = vcltq_f32(vconf, vdupq_n_f32(tof_param->MIN_CONFIDENCE));

				float32x4_t ret_conffilter = vbslq_f32(vorrq_u32(cond_ref, cond_conf), vdupq_n_f32(NAN), vdepth);
				
				/* KILL FLYING */
				float32x4_t vflyth = vdupq_n_f32(tof_param->KILL_FLYING_DELTA);

				uint32x4_t cond_vert = vandq_u32(
						vcgtq_f32(vabdq_f32(in[0][1], in[1][1]), vflyth),
						vcgtq_f32(vabdq_f32(in[2][1], in[1][1]), vflyth));
				
				uint32x4_t cond_horz = vandq_u32(
						vcgtq_f32(vabdq_f32(in[1][0], in[1][1]), vflyth),
						vcgtq_f32(vabdq_f32(in[1][2], in[1][1]), vflyth));
				
				float32x4_t ret_flying = vbslq_f32(vorrq_u32(cond_vert, cond_horz), vdupq_n_f32(NAN), ret_conffilter);

				float32x4_t cx = vld1q_f32(calib->LensLUT_x.ptr<float>() + pos0);
				float32x4_t cy = vld1q_f32(calib->LensLUT_y.ptr<float>() + pos0);
				float32x4_t cz = vld1q_f32(calib->LensLUT_z.ptr<float>() + pos0);
				
				float32x4x3_t pcl;
				pcl.val[0] = vmulq_f32(cx, ret_flying);
				pcl.val[1] = vmulq_f32(cy, ret_flying);
				pcl.val[2] = vmulq_f32(cz, ret_flying);

				vst3q_f32(ret.ptr<float>() + pos0 * 3, pcl);
			}
		}

		cv::patchNaNs(ret);
	}


	void tof_calc_distance_single(int freq, const char* src, char* dst, const ToFParam* tof_param, const CalibData* calib)
	{
		if(freq == 20)
		{
			__tof_calc_distance_single<20>(src, dst, tof_param, calib);
		}
		else if(freq == 120)
		{
			__tof_calc_distance_single<120>(src, dst, tof_param, calib);
		}
	}


	void tof_calc_distance_dual(const char* src, char* dst, const ToFParam* tof_param, const CalibData* calib)
	{
		constexpr int width = 640;
		constexpr int height = 480;
		constexpr int phase_num = 4;
		constexpr int freq_num = 2;

		constexpr float pi = std::numbers::pi_v<float>;
		constexpr float c0 = 299.792458f;
		constexpr float inv_2pi = 1.f / (2.f * pi);

		constexpr int freq_list[freq_num] = { 20, 120 };
		constexpr float maxD[freq_num] = { c0 / (2.f * freq_list[0]), c0 / (2.f * freq_list[1]) };

		constexpr int dealias_count = freq_list[1] / freq_list[0];

		cv::Mat confidence(height, width, CV_16SC1);
		cv::Mat blur_confidence(height, width, CV_16SC1);

		cv::Mat radialDepth(height, width, CV_32FC1);
		static cv::Mat radialTmp = cv::Mat::zeros(height, width, CV_32FC1);
		static cv::Mat radialMixRate(height, width, CV_32FC1);

		RAW* raw = (RAW*)src;
		RAW* Q[freq_num][phase_num] = 
		{
			{ 
				&raw[width * height * 0],
				&raw[width * height * 1],
				&raw[width * height * 2],
				&raw[width * height * 3],	
			},
			{
				&raw[width * height * 4],
				&raw[width * height * 5],
				&raw[width * height * 6],
				&raw[width * height * 7],
			},
		};

		cv::Mat Q13[freq_num] = 
		{
			cv::Mat(height, width, CV_16SC1),
			cv::Mat(height, width, CV_16SC1),
		};
		cv::Mat Q20[freq_num] = 
		{
			cv::Mat(height, width, CV_16SC1),
			cv::Mat(height, width, CV_16SC1),
		};

		cv::Mat ret(height, width, CV_32FC3, dst);

		/* CALC DIFF Q */
		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=8)
			{
				int pos = x + y * width;

				int16x8_t vconf = vdupq_n_s16(0);

				for(int f = 0; f < freq_num; ++f)
				{
					int16x8_t vq[phase_num];
					for(int p = 0; p < phase_num; ++p)
					{
						int16x8x2_t vr = vld2q_s16((int16_t*)&Q[f][p][pos]);
						int16x8_t vqa = vr.val[0];
						int16x8_t vqb = vr.val[1];
						vq[p] = vsubq_s16(vqa, vqb);
					}

					int16x8_t vq13 = vsubq_s16(vq[1], vq[3]);
					int16x8_t vq20 = vsubq_s16(vq[2], vq[0]);

					vconf = vaddq_s16(vconf, vaddq_s16(vabsq_s16(vq13), vabsq_s16(vq20)));

					vst1q_s16(Q13[f].ptr<int16_t>() + pos, vq13);
					vst1q_s16(Q20[f].ptr<int16_t>() + pos, vq20);
				}

				vst1q_s16(confidence.ptr<int16_t>() + pos, vconf);
			}
		}

		/* REFINEMENT RAW */
		#pragma omp parallel for schedule(guided)	
		for(int i = 0; i < 4; ++i)
		{
			for(int f = 0; f < freq_num; ++f)
			{
				cv::Rect roi(0, height / 4 * i, width, height / 4);
				cv::blur(Q13[f](roi), Q13[f](roi), { 3, 3 });
				cv::blur(Q20[f](roi), Q20[f](roi), { 3, 3 });
			}
		}

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos = x + y * width;

				float32x4_t new_phase[freq_num];

				/* CALC PHASE */
				for(int f = 0; f < freq_num; ++f)
				{
					int16x4_t vq13_s16 = vld1_s16(Q13[f].ptr<int16_t>() + pos);
					int16x4_t vq20_s16 = vld1_s16(Q20[f].ptr<int16_t>() + pos);

					float32x4_t vq13_f32 = vcvtq_f32_s32(vmovl_s16(vq13_s16));
					float32x4_t vq20_f32 = vcvtq_f32_s32(vmovl_s16(vq20_s16));

					float32x4_t raw_phase = vaddq_f32(vatan2q_f32(vq13_f32, vq20_f32), vdupq_n_f32(pi));
					int32x4_t scaled_raw_phase = vcvtq_s32_f32(vmulq_n_f32(raw_phase, 1023.f * inv_2pi));
					float32x4_t cyc_phase;
					for(int i = 0; i < 4; ++i)
					{
						cyc_phase[i] = calib->cyclicLUT[f][scaled_raw_phase[i]];
					}

					float32x4_t gdc_offset = vld1q_f32(calib->gradient[f].ptr<float>() + pos);

					float32x4_t tec_offset = 
						vdupq_n_f32(0.f);
						// temp_offset[i];

					new_phase[f] = vaddq_f32(cyc_phase, vaddq_f32(gdc_offset, tec_offset));
					new_phase[f] = fmodf_2pi(vaddq_f32(new_phase[f], vdupq_n_f32(2.f * pi))); 

				}

				/* DEALIAS */
				float32x4_t vdepth_lo = vmulq_n_f32(new_phase[0], maxD[0] * inv_2pi);
				float32x4_t vdepth_hi = vmulq_n_f32(new_phase[1], maxD[1] * inv_2pi);
				float32x4_t vcTp = vdupq_n_f32(maxD[1]);

				float32x4_t vd = vabdq_f32(vdepth_hi, vdepth_lo);
				float32x4_t vn = vdupq_n_f32(0.f);

				for(int i = 1; i < dealias_count; ++i)
				{
					float32x4_t vi = vdupq_n_f32((float)i);
					float32x4_t tmp = vmulq_f32(vcTp, vi);
					tmp = vaddq_f32(tmp, vdepth_hi);
					tmp = vsubq_f32(tmp, vdepth_lo);
					float32x4_t vdc = vabsq_f32(tmp);

					uint32x4_t cond = vcltq_f32(vdc, vd);
					vd = vbslq_f32(cond, vdc, vd);
					vn = vbslq_f32(cond, vi, vn);
				}

				float32x4_t dealiased = vfmaq_f32(vdepth_hi, vn, vcTp);

				vst1q_f32(radialDepth.ptr<float>() + pos, dealiased);
			}
		}



		/* TEMPORAL ANTIALIAS */
		cv::copyMakeBorder(radialDepth, radialDepth, 1, 1, 1, 1, cv::BORDER_DEFAULT);

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos0 = x + y * width;

				float32x4_t anti_aliased = vld1q_f32(radialTmp.ptr<float>() + pos0);
				float32x4_t mix_rate = vminq_f32(vld1q_f32(radialMixRate.ptr<float>() + pos0), vdupq_n_f32(0.5f));
				float32x4_t in[3][3];
				// more optimize !
				for(int i = 0; i < 3; ++i)
				{
					for(int j = 0; j < 3; ++j)
					{
						int pos1 = (x + j) + (y + i) * (width + 2);
						in[i][j] = vld1q_f32(radialDepth.ptr<float>() + pos1);
					}
				}

				float32x4_t alpha = vsubq_f32(vdupq_n_f32(1.f), mix_rate);
				float32x4_t beta = mix_rate;

				anti_aliased = vaddq_f32(vmulq_f32(vmulq_f32(anti_aliased, anti_aliased), alpha),
							 vmulq_f32(vmulq_f32(in[1][1], in[1][1]), beta));
				anti_aliased = vsqrtq_f32(anti_aliased);

				float32x4_t min_depth = vminq_f32(vminq_f32(vminq_f32(in[1][1], in[0][0]), vminq_f32(in[0][1], in[0][2])), in[1][0]);
				float32x4_t max_depth = vmaxq_f32(vmaxq_f32(vmaxq_f32(in[1][1], in[0][0]), vmaxq_f32(in[0][1], in[0][2])), in[1][0]);
				alpha = vdupq_n_f32(0.5f);
				beta = vdupq_n_f32(0.5f);

				min_depth = vaddq_f32(vmulq_f32(min_depth, alpha), 
						      vmulq_f32(vminq_f32(vminq_f32(vminq_f32(in[1][2], in[2][0]), vminq_f32(in[2][1], in[2][2])), min_depth), beta));

				max_depth = vaddq_f32(vmulq_f32(max_depth, alpha), 
						      vmulq_f32(vmaxq_f32(vmaxq_f32(vmaxq_f32(in[1][2], in[2][0]), vmaxq_f32(in[2][1], in[2][2])), max_depth), beta));

				float32x4_t pre_clamping = anti_aliased;
				anti_aliased = vminq_f32(vmaxq_f32(anti_aliased, min_depth), max_depth);

				mix_rate = vdivq_f32(vdupq_n_f32(1.f), vaddq_f32(vdupq_n_f32(1.f), vdivq_f32(vdupq_n_f32(1.f), mix_rate)));

				float32x4_t diff = vsubq_f32(anti_aliased, pre_clamping);
				float32x4_t clamp_amount = vmulq_f32(diff, diff);

				mix_rate = vfmaq_f32(mix_rate, clamp_amount, vdupq_n_f32(4.f));
				mix_rate = vminq_f32(vmaxq_f32(mix_rate, vdupq_n_f32(0.05f)), vdupq_n_f32(0.5f));

				vst1q_f32(radialTmp.ptr<float>() + pos0, anti_aliased);
				vst1q_f32(radialMixRate.ptr<float>() + pos0, mix_rate);
			}
		}

		blur_confidence = cv::abs(Q13[0]) + cv::abs(Q13[1]) + cv::abs(Q20[0]) + cv::abs(Q20[1]);


		/* POST FILTER */
		cv::copyMakeBorder(radialTmp, radialDepth, 1, 1, 1, 1, cv::BORDER_DEFAULT);

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos0 = x + y * width;
				float32x4_t in[3][3];
				for(int i = 0; i < 3; ++i)
				{
					for(int j = 0; j < 3; ++j)
					{
						int pos1 = (x + j) + (y + i) * (width + 2);
						in[i][j] = vld1q_f32(radialDepth.ptr<float>() + pos1);
					}
				}
				
				/* CONFIDENCE FILTER*/
				float32x4_t vconf = vcvtq_f32_s32(vmovl_s16(vld1_s16(blur_confidence.ptr<int16_t>() + pos0)));
				float32x4_t vdepth = in[1][1];
				float32x4_t vinv_d2 = vdivq_f32(vdupq_n_f32(1.f), vmulq_f32(vdepth, vdepth));

				uint32x4_t cond_ref = vcltq_f32(vconf, vmulq_f32(vdupq_n_f32(tof_param->MIN_REFLECTANCE), vinv_d2));
				uint32x4_t cond_conf = vcltq_f32(vconf, vdupq_n_f32(tof_param->MIN_CONFIDENCE));

				float32x4_t ret_conffilter = vbslq_f32(vorrq_u32(cond_ref, cond_conf), vdupq_n_f32(NAN), vdepth);
				
				/* KILL FLYING */
				float32x4_t vflyth = vdupq_n_f32(tof_param->KILL_FLYING_DELTA);

				uint32x4_t cond_vert = vandq_u32(
						vcgtq_f32(vabdq_f32(in[0][1], in[1][1]), vflyth),
						vcgtq_f32(vabdq_f32(in[2][1], in[1][1]), vflyth));
				
				uint32x4_t cond_horz = vandq_u32(
						vcgtq_f32(vabdq_f32(in[1][0], in[1][1]), vflyth),
						vcgtq_f32(vabdq_f32(in[1][2], in[1][1]), vflyth));
				
				float32x4_t ret_flying = vbslq_f32(vorrq_u32(cond_vert, cond_horz), vdupq_n_f32(NAN), ret_conffilter);

				float32x4_t cx = vld1q_f32(calib->LensLUT_x.ptr<float>() + pos0);
				float32x4_t cy = vld1q_f32(calib->LensLUT_y.ptr<float>() + pos0);
				float32x4_t cz = vld1q_f32(calib->LensLUT_z.ptr<float>() + pos0);
				
				float32x4x3_t pcl;
				pcl.val[0] = vmulq_f32(cx, ret_flying);
				pcl.val[1] = vmulq_f32(cy, ret_flying);
				pcl.val[2] = vmulq_f32(cz, ret_flying);

				vst3q_f32(ret.ptr<float>() + pos0 * 3, pcl);
			}
		}

		cv::patchNaNs(ret);
	}


	void tof_calc_distance_hdr(const char* src, char* dst, const ToFParam* tof_param, const CalibData* calib)
	{
		constexpr int width = 640;
		constexpr int height = 480;
		constexpr int phase_num = 4;
		constexpr int freq_num = 3;

		constexpr float pi = std::numbers::pi_v<float>;
		constexpr float c0 = 299.792458f;
		constexpr float inv_2pi = 1.f / (2.f * pi);

		constexpr int freq_list[freq_num] = { 20, 120, 120 };
		constexpr float maxD[freq_num] = 
		{ c0 / (2.f * freq_list[0]), c0 / (2.f * freq_list[1]), c0 / (2.f * freq_list[2]) };

		constexpr int dealias_count = freq_list[1] / freq_list[0];

		cv::Mat confidence(height, width, CV_16SC1);
		cv::Mat blur_confidence(height, width, CV_16SC1);

		cv::Mat radialDepth[2] = 
		{
			cv::Mat(height, width, CV_32FC1),
			cv::Mat(height, width, CV_32FC1),
		};

		static cv::Mat radialTmp[2] =
		{       
			cv::Mat::zeros(height, width, CV_32FC1),
			cv::Mat::zeros(height, width, CV_32FC1)
		};

		static cv::Mat radialMixRate[2] =
		{
			cv::Mat(height, width, CV_32FC1),
			cv::Mat(height, width, CV_32FC1),
		};

		RAW* raw = (RAW*)src;
		RAW* Q[freq_num][phase_num] = 
		{
			{ 
				&raw[width * height * 0],
				&raw[width * height * 1],
				&raw[width * height * 2],
				&raw[width * height * 3],	
			},
			{
				&raw[width * height * 4],
				&raw[width * height * 5],
				&raw[width * height * 6],
				&raw[width * height * 7],
			},
			{
				&raw[width * height * 8],
				&raw[width * height * 9],
				&raw[width * height * 10],
				&raw[width * height * 11],
			},

		};

		cv::Mat Q13[freq_num] = 
		{
			cv::Mat(height, width, CV_16SC1),
			cv::Mat(height, width, CV_16SC1),
			cv::Mat(height, width, CV_16SC1),
		};
		cv::Mat Q20[freq_num] = 
		{
			cv::Mat(height, width, CV_16SC1),
			cv::Mat(height, width, CV_16SC1),
			cv::Mat(height, width, CV_16SC1),
		};

		cv::Mat ret(height, width, CV_32FC3, dst);

		/* CALC DIFF Q */
		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=8)
			{
				int pos = x + y * width;

				int16x8_t vconf = vdupq_n_s16(0);

				for(int f = 0; f < freq_num; ++f)
				{
					int16x8_t vq[phase_num];
					for(int p = 0; p < phase_num; ++p)
					{
						int16x8x2_t vr = vld2q_s16((int16_t*)&Q[f][p][pos]);
						int16x8_t vqa = vr.val[0];
						int16x8_t vqb = vr.val[1];
						vqa = vandq_s16(vqa, vdupq_n_s16(0b0111'1111'1111));
						vqb = vandq_s16(vqb, vdupq_n_s16(0b0111'1111'1111));
						vq[p] = vsubq_s16(vqa, vqb);
					}

					int16x8_t vq13 = vsubq_s16(vq[1], vq[3]);
					int16x8_t vq20 = vsubq_s16(vq[2], vq[0]);

					vconf = vaddq_s16(vconf, vaddq_s16(vabsq_s16(vq13), vabsq_s16(vq20)));

					vst1q_s16(Q13[f].ptr<int16_t>() + pos, vq13);
					vst1q_s16(Q20[f].ptr<int16_t>() + pos, vq20);
				}

				vst1q_s16(confidence.ptr<int16_t>() + pos, vconf);
			}
		}

		/* REFINEMENT RAW */
		#pragma omp parallel for schedule(guided)	
		for(int i = 0; i < 4; ++i)
		{
			for(int f = 0; f < freq_num; ++f)
			{
				cv::Rect roi(0, height / 4 * i, width, height / 4);
				cv::blur(Q13[f](roi), Q13[f](roi), { 3, 3 });
				cv::blur(Q20[f](roi), Q20[f](roi), { 3, 3 });
			}
		}

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos = x + y * width;

				float32x4_t new_phase[freq_num];

				/* CALC PHASE */
				for(int f = 0; f < freq_num; ++f)
				{
					int c = f >= 1 ? 1 : 0;
					int16x4_t vq13_s16 = vld1_s16(Q13[f].ptr<int16_t>() + pos);
					int16x4_t vq20_s16 = vld1_s16(Q20[f].ptr<int16_t>() + pos);

					float32x4_t vq13_f32 = vcvtq_f32_s32(vmovl_s16(vq13_s16));
					float32x4_t vq20_f32 = vcvtq_f32_s32(vmovl_s16(vq20_s16));

					float32x4_t raw_phase = vaddq_f32(vatan2q_f32(vq13_f32, vq20_f32), vdupq_n_f32(pi));
					int32x4_t scaled_raw_phase = vcvtq_s32_f32(vmulq_n_f32(raw_phase, 1023.f * inv_2pi));
					float32x4_t cyc_phase;
					for(int i = 0; i < 4; ++i)
					{
						cyc_phase[i] = calib->cyclicLUT[c][scaled_raw_phase[i]];
					}

					float32x4_t gdc_offset = vld1q_f32(calib->gradient[c].ptr<float>() + pos);

					float32x4_t tec_offset = 
						vdupq_n_f32(0.f);
						// temp_offset[i];

					new_phase[f] = vaddq_f32(cyc_phase, vaddq_f32(gdc_offset, tec_offset));
					new_phase[f] = fmodf_2pi(vaddq_f32(new_phase[f], vdupq_n_f32(2.f * pi))); 

				}

				/* DEALIAS */
				float32x4_t vdepth_lo = vmulq_n_f32(new_phase[0], maxD[0] * inv_2pi);
				float32x4_t vdepth_hi1 = vmulq_n_f32(new_phase[1], maxD[1] * inv_2pi);
				float32x4_t vdepth_hi2 = vmulq_n_f32(new_phase[2], maxD[1] * inv_2pi);
				float32x4_t vcTp = vdupq_n_f32(maxD[1]);

				float32x4_t vd1 = vabdq_f32(vdepth_hi1, vdepth_lo);
				float32x4_t vd2 = vabdq_f32(vdepth_hi2, vdepth_lo);
				float32x4_t vn1 = vdupq_n_f32(0.f);
				float32x4_t vn2 = vdupq_n_f32(0.f);

				for(int i = 1; i < dealias_count; ++i)
				{
					float32x4_t vi1 = vdupq_n_f32((float)i);
					float32x4_t vi2 = vdupq_n_f32((float)i);
					float32x4_t tmp1 = vmulq_f32(vcTp, vi1);
					float32x4_t tmp2 = vmulq_f32(vcTp, vi2);
					tmp1 = vaddq_f32(tmp1, vdepth_hi1);
					tmp2 = vaddq_f32(tmp2, vdepth_hi2);
					tmp1 = vsubq_f32(tmp1, vdepth_lo);
					tmp2 = vsubq_f32(tmp2, vdepth_lo);
					float32x4_t vdc1 = vabsq_f32(tmp1);
					float32x4_t vdc2 = vabsq_f32(tmp2);

					uint32x4_t cond1 = vcltq_f32(vdc1, vd1);
					uint32x4_t cond2 = vcltq_f32(vdc2, vd2);
					vd1 = vbslq_f32(cond1, vdc1, vd1);
					vd2 = vbslq_f32(cond2, vdc2, vd2);
					vn1 = vbslq_f32(cond1, vi1, vn1);
					vn2 = vbslq_f32(cond2, vi2, vn2);
				}

				float32x4_t dealiased1 = vfmaq_f32(vdepth_hi1, vn1, vcTp);
				float32x4_t dealiased2 = vfmaq_f32(vdepth_hi2, vn2, vcTp);

				vst1q_f32(radialDepth[0].ptr<float>() + pos, dealiased1);
				vst1q_f32(radialDepth[1].ptr<float>() + pos, dealiased2);
			}
		}

		/* TEMPORAL ANTIALIAS */
		cv::copyMakeBorder(radialDepth[0], radialDepth[0], 1, 1, 1, 1, cv::BORDER_DEFAULT);
		cv::copyMakeBorder(radialDepth[1], radialDepth[1], 1, 1, 1, 1, cv::BORDER_DEFAULT);

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos0 = x + y * width;

				for(int f = 0; f < 2; ++f)
				{
					float32x4_t anti_aliased = vld1q_f32(radialTmp[f].ptr<float>() + pos0);
					float32x4_t mix_rate = vminq_f32(vld1q_f32(radialMixRate[f].ptr<float>() + pos0), vdupq_n_f32(0.5f));
					float32x4_t in[3][3];
					// more optimize !
					for(int i = 0; i < 3; ++i)
					{
						for(int j = 0; j < 3; ++j)
						{
							int pos1 = (x + j) + (y + i) * (width + 2);
							in[i][j] = vld1q_f32(radialDepth[f].ptr<float>() + pos1);
						}
					}

					float32x4_t alpha = vsubq_f32(vdupq_n_f32(1.f), mix_rate);
					float32x4_t beta = mix_rate;

					anti_aliased = vaddq_f32(vmulq_f32(vmulq_f32(anti_aliased, anti_aliased), alpha),
								 vmulq_f32(vmulq_f32(in[1][1], in[1][1]), beta));
					anti_aliased = vsqrtq_f32(anti_aliased);

					float32x4_t min_depth = vminq_f32(vminq_f32(vminq_f32(in[1][1], in[0][0]), vminq_f32(in[0][1], in[0][2])), in[1][0]);
					float32x4_t max_depth = vmaxq_f32(vmaxq_f32(vmaxq_f32(in[1][1], in[0][0]), vmaxq_f32(in[0][1], in[0][2])), in[1][0]);
					alpha = vdupq_n_f32(0.5f);
					beta = vdupq_n_f32(0.5f);

					min_depth = vaddq_f32(vmulq_f32(min_depth, alpha), 
							      vmulq_f32(vminq_f32(vminq_f32(vminq_f32(in[1][2], in[2][0]), vminq_f32(in[2][1], in[2][2])), min_depth), beta));

					max_depth = vaddq_f32(vmulq_f32(max_depth, alpha), 
							      vmulq_f32(vmaxq_f32(vmaxq_f32(vmaxq_f32(in[1][2], in[2][0]), vmaxq_f32(in[2][1], in[2][2])), max_depth), beta));

					float32x4_t pre_clamping = anti_aliased;
					anti_aliased = vminq_f32(vmaxq_f32(anti_aliased, min_depth), max_depth);

					mix_rate = vdivq_f32(vdupq_n_f32(1.f), vaddq_f32(vdupq_n_f32(1.f), vdivq_f32(vdupq_n_f32(1.f), mix_rate)));

					float32x4_t diff = vsubq_f32(anti_aliased, pre_clamping);
					float32x4_t clamp_amount = vmulq_f32(diff, diff);

					mix_rate = vfmaq_f32(mix_rate, clamp_amount, vdupq_n_f32(4.f));
					mix_rate = vminq_f32(vmaxq_f32(mix_rate, vdupq_n_f32(0.05f)), vdupq_n_f32(0.5f));

					vst1q_f32(radialTmp[f].ptr<float>() + pos0, anti_aliased);
					vst1q_f32(radialMixRate[f].ptr<float>() + pos0, mix_rate);
				}
			}
		}

		/* Mix */
		cv::Mat Mix(height, width, CV_32FC1);
		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos = x + y * width;
				float32x4_t L = vld1q_f32(radialTmp[0].ptr<float>() + pos);
				float32x4_t S = vld1q_f32(radialTmp[1].ptr<float>() + pos);

				vst1q_f32(Mix.ptr<float>() + pos, vbslq_f32(vcgtq_f32(S, vdupq_n_f32(0.25f)), L, S));
				//vst1q_f32(Mix.ptr<float>() + pos, vmaxq_f32(S, L));
			}
		}
		
		blur_confidence =
		       	cv::abs(Q13[0]) + cv::abs(Q13[1]) + cv::abs(Q13[2]) +
		       	cv::abs(Q20[0]) + cv::abs(Q20[1]) + cv::abs(Q20[2]);


		/* POST FILTER */
		cv::copyMakeBorder(Mix, Mix, 1, 1, 1, 1, cv::BORDER_DEFAULT);

		#pragma omp parallel for schedule(guided)
		for(int y = 0; y < height; ++y)
		{
			for(int x = 0; x < width; x+=4)
			{
				int pos0 = x + y * width;
				float32x4_t in[3][3];
				for(int i = 0; i < 3; ++i)
				{
					for(int j = 0; j < 3; ++j)
					{
						int pos1 = (x + j) + (y + i) * (width + 2);
						in[i][j] = vld1q_f32(Mix.ptr<float>() + pos1);
					}
				}
				
				/* CONFIDENCE FILTER*/
				float32x4_t vconf = vcvtq_f32_s32(vmovl_s16(vld1_s16(blur_confidence.ptr<int16_t>() + pos0)));
				float32x4_t vdepth = in[1][1];
				float32x4_t vinv_d2 = vdivq_f32(vdupq_n_f32(1.f), vmulq_f32(vdepth, vdepth));

				uint32x4_t cond_ref = vcltq_f32(vconf, vmulq_f32(vdupq_n_f32(tof_param->MIN_REFLECTANCE), vinv_d2));
				uint32x4_t cond_conf = vcltq_f32(vconf, vdupq_n_f32(tof_param->MIN_CONFIDENCE));

				float32x4_t ret_conffilter = vbslq_f32(vorrq_u32(cond_ref, cond_conf), vdupq_n_f32(NAN), vdepth);
				
				/* KILL FLYING */
				float32x4_t vflyth = vdupq_n_f32(tof_param->KILL_FLYING_DELTA);

				uint32x4_t cond_vert = vandq_u32(
						vcgtq_f32(vabdq_f32(in[0][1], in[1][1]), vflyth),
						vcgtq_f32(vabdq_f32(in[2][1], in[1][1]), vflyth));
				
				uint32x4_t cond_horz = vandq_u32(
						vcgtq_f32(vabdq_f32(in[1][0], in[1][1]), vflyth),
						vcgtq_f32(vabdq_f32(in[1][2], in[1][1]), vflyth));
				
				float32x4_t ret_flying = vbslq_f32(vorrq_u32(cond_vert, cond_horz), vdupq_n_f32(NAN), ret_conffilter);

				float32x4_t cx = vld1q_f32(calib->LensLUT_x.ptr<float>() + pos0);
				float32x4_t cy = vld1q_f32(calib->LensLUT_y.ptr<float>() + pos0);
				float32x4_t cz = vld1q_f32(calib->LensLUT_z.ptr<float>() + pos0);
				
				float32x4x3_t pcl;
				pcl.val[0] = vmulq_f32(cx, ret_flying);
				pcl.val[1] = vmulq_f32(cy, ret_flying);
				pcl.val[2] = vmulq_f32(cz, ret_flying);

				vst3q_f32(ret.ptr<float>() + pos0 * 3, pcl);
			}
		}

		cv::patchNaNs(ret);
	}


	void rgbd_fusion(const char* tof, const char* yuv, char* dst, const ToFParam* tof_param, const CalibData* calib, const FusionData* fusion)
	{
		constexpr int rgb_width = 1920;
		constexpr int rgb_height = 1080;

		constexpr int tof_width = 640;
		constexpr int tof_height = 480;

		cv::Mat tof_tmp(tof_height, tof_width, CV_32FC3);
		tof_calc_distance_dual(tof, tof_tmp.ptr<char>(), tof_param, calib);

		cv::Mat yuvMat(rgb_height, rgb_width, CV_8UC2, (char*)yuv);
		cv::Mat bgrMat;
		cv::cvtColor(yuvMat, bgrMat, cv::COLOR_YUV2BGR_YUYV);

		cv::Mat1f R(fusion->R);
		cv::Mat1f t(fusion->t);
		cv::Mat1f camMat(fusion->cameraMatrix);

		float32x4_t vR[3][3];
		float32x4_t vt[3];
		
		for(int i = 0; i < 3; ++i)
		{
			for(int j = 0; j < 3; ++j)
			{
				vR[i][j] = vdupq_n_f32(R(i, j));
			}
			vt[i] = vdupq_n_f32(t(i));
		}

		float32x4_t vfx = vdupq_n_f32(camMat(0, 0));
		float32x4_t vfy = vdupq_n_f32(camMat(1, 1));
		float32x4_t vcx = vdupq_n_f32(camMat(0, 2));
		float32x4_t vcy = vdupq_n_f32(camMat(1, 2));

		RGBD* ret = (RGBD*)dst;

		#pragma omp parallel for schedule(guided)	
		for(int v = 0; v < tof_height; ++v)
		{
			float maxU = -1.f; 

			for(int u = tof_width - 4; u >=0; u-=4)
			{
				int pos = u + v * tof_width;

				float32x4x3_t vxyz = vld3q_f32(tof_tmp.ptr<float>() + pos * 3);
				float32x4_t vx = vxyz.val[0];
				float32x4_t vy = vxyz.val[1];
				float32x4_t vz = vxyz.val[2];

				float32x4_t ax = vfmaq_f32(vfmaq_f32(vfmaq_f32(vt[0], vR[0][0], vx), vR[0][1], vy), vR[0][2], vz);
				float32x4_t ay = vfmaq_f32(vfmaq_f32(vfmaq_f32(vt[1], vR[1][0], vx), vR[1][1], vy), vR[1][2], vz);
				float32x4_t az = vfmaq_f32(vfmaq_f32(vfmaq_f32(vt[2], vR[2][0], vx), vR[2][1], vy), vR[2][2], vz);

				float32x4_t fx = vfmaq_f32(vcx, ax, vdivq_f32(vfx, az));
				float32x4_t fy = vfmaq_f32(vcy, ay, vdivq_f32(vfy, az));

				int32x4_t ix = vcvtq_s32_f32(fx);
				int32x4_t iy = vcvtq_s32_f32(fy);

				RGBD pix[4]{};
				for(int i = 3; i >= 0; --i)
				{
					pix[i].x = vx[i];
					pix[i].y = vy[i];
					pix[i].z = vz[i];

					float U = rgb_width - fx[i];
				
					if(pix[i].z > 0.f)
					{
						if(U < maxU)
						{
							continue;
						}
						else
						{
							maxU = U;
						}
					

						if(0 < ix[i] && ix[i] < rgb_width && 0 < iy[i] && iy[i] < rgb_height)
						{
							cv::Vec3b c = bgrMat.at<cv::Vec3b>(iy[i], ix[i]);
							pix[i].b = c[0];
							pix[i].g = c[1];
							pix[i].r = c[2];
						}
					}
				}
				
				memcpy(ret + u + v * tof_width, &pix[0], sizeof(pix));

			}
		}

	}


} /* namespace cis */
