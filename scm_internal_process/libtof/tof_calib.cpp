#include <cmath>
#include <cstring>
#include <cstdio>
#include <numbers>

#include "tof.h"

namespace
{
	constexpr int width = 640;
	constexpr int height = 480;
	constexpr float pi = std::numbers::pi_v<float>;
}

namespace cis
{
	namespace cyclic
	{
		std::vector<float> compute(const float* fcoef)
		{
			auto F = [](const float* fcoef, float x, float x0)
			{
				return fcoef[0] * cos(2.f * x) + fcoef[2] * sin(2.f * x) + fcoef[1] * cos(4.f * x) + fcoef[3] * sin(4.f * x) + x - x0;
			};

			auto FP = [](const float* fcoef, float x)
                	{
                       		return 1 - 2.f * fcoef[0] * sin(2.f * x) + 2.f * fcoef[2] * cos(2.f * x) - 4.f * fcoef[1] * sin(4.f * x) + 4.f * fcoef[3] * cos(4.f * x);
                	};

			constexpr int lut_size = 1024;
			std::vector<float> ret(lut_size);

			constexpr float rep_count = 5;
			for(int i = 0; i < lut_size; ++i)
			{
				float phi = ((float)i / lut_size) * 2.f * pi;
				float phi0 = phi;

				for(int j = 0; j < rep_count; ++j)
				{
					phi = phi - F(fcoef, phi, phi0) / FP(fcoef, phi);
				}
				ret[i] = phi;
			}

			return ret;
		}

		void init(CalibData* calib, const std::string& cyclic_files_path)
		{
			cv::FileStorage fs20(cyclic_files_path + "cyclic_20MHz.xml", cv::FileStorage::READ);
			cv::FileStorage fs120(cyclic_files_path + "cyclic_120MHz.xml", cv::FileStorage::READ);

			cv::Mat c20, c120;
			fs20["coef"] >> c20;
			fs120["coef"] >> c120;

			float cyc_lo_coef[4];
			float cyc_hi_coef[4];

			for(int i = 0; i < 2; ++i)
			{
				for(int j = 0; j < 2; ++j)
				{
					cyc_lo_coef[i * 2 + j] = c20.at<float>(i, j);
					cyc_hi_coef[i * 2 + j] = c120.at<float>(i, j);
				}
			}

			calib->cyclicLUT[0] = compute(cyc_lo_coef);
			calib->cyclicLUT[1] = compute(cyc_hi_coef);
		}
	}

	namespace gradient
	{
		cv::Mat compute(const float* fcoef)
		{
			constexpr float mean_x = width / 2.f;
			constexpr float mean_y = height / 2.f;
			constexpr float inv_std_x = 1.f / 184.897044f;
			constexpr float inv_std_y = 1.f / 138.709230f;

			cv::Mat ret(height, width, CV_32FC1);

			for(int y = 0; y < height; ++y)
			{
				for(int x = 0; x < width; ++x)
				{
					float xv = (x - mean_x) * inv_std_x;
					float yv = (y - mean_y) * inv_std_y;

					ret.at<float>(y, x) =
						fcoef[0] +
						fcoef[1] * xv +
						fcoef[2] * yv +
						fcoef[3] * xv * xv +
						fcoef[4] * xv * yv +
						fcoef[5] * yv * yv;
				}
			}

			return ret;
		}

		void init(CalibData* calib, const std::string& grad_files_path)
		{
			cv::FileStorage fs20(grad_files_path + "gradient_20MHz.xml", cv::FileStorage::READ);
			cv::FileStorage fs120(grad_files_path + "gradient_120MHz.xml", cv::FileStorage::READ);

			cv::Mat c20, c120;
			fs20["coef"] >> c20;
			fs120["coef"] >> c120;

			float gdc_lo_coef[6];
			float gdc_hi_coef[6];

			for(int i = 0; i < 6; ++i)
			{
				gdc_lo_coef[i] = c20.at<float>(i);
				gdc_hi_coef[i] = c120.at<float>(i);
			}

			calib->gradient[0] = compute(gdc_lo_coef);
			calib->gradient[1] = compute(gdc_hi_coef);
		}
	}

	namespace temperature
	{
		void init(CalibData* calib)
		{
			/* TODO : implement here */
		}
	}

	namespace lens
	{
		constexpr int CONFIGURATION_NUMBER_OF_DEPTH_INTRINSICS_ELEMENTS = 9;
		constexpr int SEP_LUT_WIDTH_ACT = 40;
		constexpr int SEP_LUT_HEIGHT_ACT = 30;
		constexpr int SEP_LUT_WIDTH = 42;
		constexpr int SEP_LUT_HEIGHT = 32;

		constexpr float B2PI = 32768.f;
		constexpr float BINARY_RADIAN_2PI = B2PI;

		constexpr int VGA_WIDTH = 640;
		constexpr int VGA_HEIGHT = 480;
		constexpr int UVGA_WIDTH = 1280;
		constexpr int UVGA_HEIGHT = 960;
		constexpr int POST_CAMERA_RAY_LUT2D_STRIDE = SEP_LUT_WIDTH / 2;

		struct LensLUT_Detail
		{
			int x[SEP_LUT_WIDTH * SEP_LUT_HEIGHT];
			int y[SEP_LUT_WIDTH * SEP_LUT_HEIGHT];
			int z[SEP_LUT_WIDTH * SEP_LUT_HEIGHT];
		};

		cv::Point2d distort_brown(cv::Point2d u_est, const cv::Mat& distCoeffs)
		{
			double x = u_est.x;
			double y = u_est.y;

			double r2 = x * x + y * y;
			double r4 = r2 * r2;
			double r6 = r4 * r2;

			double k1 = distCoeffs.at<double>(0);
			double k2 = distCoeffs.at<double>(1);
			double p1 = distCoeffs.at<double>(2);
			double p2 = distCoeffs.at<double>(3);
			double k3 = distCoeffs.at<double>(4);

			cv::Point2d pret;
			pret.x = x * (1.0 + k1 * r2 + k2 * r4 + k3 * r6) + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
			pret.y = y * (1.0 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

			return pret;
		}


		void undistort_cost_brown(cv::Point2d d,
					  cv::Point2d u_est,
					  const cv::Mat& distCoeffs,
					  cv::Point2d* perr,
					  double* pcost)
		{
			cv::Point2d u_est_d = distort_brown(u_est, distCoeffs);
			perr->x = d.x - u_est_d.x;
			perr->y = d.y - u_est_d.y;
			*pcost = std::hypot(perr->x, perr->y);
		}


		cv::Point3d undistort_jacobian_brown(cv::Point2d u_est,
						     const cv::Mat& distCoeffs)
		{
			double x = u_est.x;
			double y = u_est.y;

			double r2 = x * x + y * y;
			double r4 = r2 * r2;
			double r6 = r4 * r2;

			double k1 = distCoeffs.at<double>(0);
			double k2 = distCoeffs.at<double>(1);
			double p1 = distCoeffs.at<double>(2);
			double p2 = distCoeffs.at<double>(3);
			double k3 = distCoeffs.at<double>(4);

			double g = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
			double dg = k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4;

			double p1y = p1 * y;
			double p2x = p2 * x;

			cv::Point3d pjacobian;
			pjacobian.z = 2.0 * (x * y * dg + p1 * x + p2 * y);
			pjacobian.y = g + 2.0 * y * y * dg + 6.0 * p1y + 2.0 * p2x;
			pjacobian.x = g + 2.0 * x * x * dg + 2.0 * p1y + 6.0 * p2x;
			return pjacobian;
		}


		bool undistort_evaluate_brown(double lambda,
					      cv::Point2d JTe,
					      cv::Point3d JTJ,
					      cv::Point2d d,
					      const cv::Mat& distCoeffs,
					      cv::Point2d* pu_est,
					      cv::Point2d* perr,
					      double* pcost)
		{
			cv::Point3d A;
			A.z = JTJ.z;
			A.x = JTJ.x * (1.0 + lambda);
			A.y = JTJ.y * (1.0 + lambda);

			double detA = A.x * A.y - A.z * A.z;

			cv::Point2d delta;
			delta.x = (A.y * JTe.x - A.z * JTe.y) / detA;
			delta.y = (-A.z * JTe.x + A.x * JTe.y) / detA;

			cv::Point2d new_u_est;
			new_u_est.x = pu_est->x + delta.x;
			new_u_est.y = pu_est->y + delta.y;

			double new_cost;
			cv::Point2d new_err;
			undistort_cost_brown(d, new_u_est, distCoeffs, &new_err, &new_cost);

			if(new_cost < *pcost)
			{
				*pu_est = new_u_est;
				*pcost = new_cost;
				*perr = new_err;
				return true;
			}

			return false;
		}


		void calc_radial_to_cartesian_projection_estimate(const cv::Mat& cameraMatrix,
								  const cv::Mat& distCoeffs,
								  float* estimate_x,
								  float* estimate_y,
								  int w, int h)
		{
			double fx = cameraMatrix.at<double>(0, 0);
			double fy = cameraMatrix.at<double>(1, 1);
			double cx = cameraMatrix.at<double>(0, 2);
			double cy = cameraMatrix.at<double>(1, 2);

			double prec_1fx = 1.0 / fx;
			double prec_1fy = 1.0 / fy;

			bool has_reached_stop_criterion = false;
			constexpr int max_iteration = 100;

			double lambda = 1.0;

			constexpr double v = 1.1;

			constexpr double stop_threshold = 1.0e-11;

			cv::Point2d d;
			cv::Point2d u_est;
			d.x = (w - cx) * prec_1fx;
			d.y = (h - cy) * prec_1fy;
			u_est = d;

			cv::Point2d previous_err{};
			double previous_cost = 0.0;
			undistort_cost_brown(d, u_est, distCoeffs, &previous_err, &previous_cost);

			for(int i = 0; i < max_iteration && !has_reached_stop_criterion; ++i)
			{
				double current_cost = previous_cost;

				cv::Point3d J = undistort_jacobian_brown(u_est, distCoeffs);

				double c2 = J.z * J.z;

				cv::Point3d JTJ;
				JTJ.x = J.x * J.x + c2;
				JTJ.y = J.y * J.y + c2;
				JTJ.z = (J.x + J.y) * J.z;

				cv::Point2d JTe;
				JTe.x = J.x * previous_err.x + J.z * previous_err.y;
				JTe.y = J.z * previous_err.x + J.y * previous_err.y;

				if(undistort_evaluate_brown(lambda / v, JTe, JTJ, d, distCoeffs, &u_est, &previous_err, &previous_cost))
				{
					lambda = lambda / v;
				}
				else if(undistort_evaluate_brown(lambda, JTe, JTJ, d, distCoeffs, &u_est, &previous_err, &previous_cost))
				{
					lambda *= v * v;
				}

				has_reached_stop_criterion = (previous_cost - current_cost) <= stop_threshold * previous_cost;
				previous_cost = current_cost;
			}

			*estimate_x = (float)(u_est.x);
			*estimate_y = (float)(u_est.y);
		}


		void calc_radial_to_cartesian_vector_field_float(float estimate_x,
								 float estimate_y,
								 float* xptr,
								 float* yptr,
								 float* zptr)
		{
			cv::Point2d u_est;
			u_est.x = estimate_x;
			u_est.y = estimate_y;
			float norm = (float)std::hypot(u_est.x, u_est.y, 1.0);

			norm *= BINARY_RADIAN_2PI;

			float range_on_norm = 1.f / norm;
			*xptr = (float)(u_est.x * range_on_norm);
			*yptr = (float)(u_est.y * range_on_norm);
			*zptr = range_on_norm;
		}


		cv::Mat datapath_processor_scale_intrinsics_for_sep(const cv::Mat& cameraMatrix_in,
								    uint32_t width_out,
								    uint32_t height_out)
		{
			uint32_t scale_base_width = width_out;
			uint32_t scale_base_height = height_out;

			float cx = (float)cameraMatrix_in.at<double>(0, 2);
			constexpr uint32_t min_calib_width = VGA_WIDTH / 4;
			constexpr uint32_t min_calib_height = VGA_HEIGHT / 4;
			constexpr uint32_t min_calib_cx = min_calib_width / 2;

			uint32_t capture_integer_factor = (uint32_t)exp2f(roundf(log2f(cx / min_calib_cx)));

			uint32_t intrinsics_in_width = min_calib_width * capture_integer_factor;
			uint32_t intrinsics_in_height = min_calib_height * capture_integer_factor;

			float scale_factor_width = (float)(scale_base_width) / intrinsics_in_width;
			float scale_factor_height = (float)(scale_base_height) / intrinsics_in_height;

			cv::Mat cameraMatrix_out;
			cameraMatrix_in.copyTo(cameraMatrix_out);

			cameraMatrix_out.at<double>(0, 0) *= scale_factor_width;
			cameraMatrix_out.at<double>(1, 1) *= scale_factor_height;
			cameraMatrix_out.at<double>(0, 2) *= scale_factor_width;
			cameraMatrix_out.at<double>(1, 2) *= scale_factor_height;

			return cameraMatrix_out;
		}

		void calc_lens_lut(const cv::Mat& cameraMatrix,
				   const cv::Mat& distCoeffs,
				   LensLUT_Detail* lut)
		{
			constexpr int width = UVGA_WIDTH;
			constexpr int height = UVGA_HEIGHT;

			float estimate_x, estimate_y;
			float lut_x, lut_y, lut_z;

			cv::Mat cameraMatrix_scaled = datapath_processor_scale_intrinsics_for_sep(cameraMatrix, width, height);

			constexpr int x_thinning = width / SEP_LUT_WIDTH_ACT;
			constexpr int y_thinning = height / SEP_LUT_HEIGHT_ACT;

			constexpr float coef_xy = BINARY_RADIAN_2PI * (1 << 19);
			constexpr float coef_z = BINARY_RADIAN_2PI * (1 << 14);

			for(int y = 0; y < SEP_LUT_HEIGHT; ++y)
			{
				for(int x = 0; x < SEP_LUT_WIDTH; ++x)
				{
					int ref_x = (x < SEP_LUT_WIDTH_ACT) ? (x * x_thinning) : width - 1;
					int ref_y = (y < SEP_LUT_HEIGHT_ACT) ? (y * y_thinning) : height - 1;
					int idx = (y & ~1) * (POST_CAMERA_RAY_LUT2D_STRIDE << 1) + ((y & 1) << 1) + ((x >> 1) << 2) + (x & 1);

					calc_radial_to_cartesian_projection_estimate(cameraMatrix_scaled, distCoeffs, &estimate_x, &estimate_y, ref_x, ref_y);

					calc_radial_to_cartesian_vector_field_float(estimate_x, estimate_y, &lut_x, &lut_y, &lut_z);

					lut->x[idx] = (int)nearbyintf(lut_x * coef_xy);
					lut->y[idx] = (int)nearbyintf(lut_y * coef_xy);
					lut->z[idx] = (int)nearbyintf(lut_z * coef_z);

					// X Y : s0.19
					lut->x[idx] = lut->x[idx] > 0x7FFFF ? 0x7FFFF : (lut->x[idx] < -0x80000 ? -0x80000 : lut->x[idx]);
					lut->y[idx] = lut->y[idx] > 0x7FFFF ? 0x7FFFF : (lut->y[idx] < -0x80000 ? -0x80000 : lut->y[idx]);
					lut->x[idx] &= 0xFFFFF;
					lut->y[idx] &= 0xFFFFF;

					// Z : u0.14
					lut->z[idx] = lut->z[idx] > 0x3FFF ? 0x3FFF : lut->z[idx];
				}
			}
		}

		struct LensLUT
		{
			int x : 20;
			int y : 20;
			int z : 15;
		};


		void init(CalibData* calib, const std::string& cam_param_files_path)
		{
			calib->LensLUT_x = cv::Mat::zeros(height, width, CV_32FC1);
			calib->LensLUT_y = cv::Mat::zeros(height, width, CV_32FC1);
			calib->LensLUT_z = cv::Mat::zeros(height, width, CV_32FC1);

			cv::Mat cameraMatrix, distCoeffs;
			cv::FileStorage fs(cam_param_files_path + "tof_internal.xml", cv::FileStorage::READ);
			fs["cameraMatrix"] >> cameraMatrix;
			fs["distCoeffs"] >> distCoeffs;

			LensLUT_Detail* detail = new LensLUT_Detail;
			calc_lens_lut(cameraMatrix, distCoeffs, detail);

			const int block_w = SEP_LUT_WIDTH;
			const int block_h = SEP_LUT_HEIGHT;

			cv::Mat tx(block_h * block_w, 1, CV_32FC1);
			cv::Mat ty(block_h * block_w, 1, CV_32FC1);
			cv::Mat tz(block_h * block_w, 1, CV_32FC1);

			for(int v = 0; v < block_h; ++v)
			{
				for(int u = 0; u < block_w; ++u)
				{
					LensLUT val;
					val.x = *(reinterpret_cast<int*>(detail->x) + u + v * block_w);
					val.y = *(reinterpret_cast<int*>(detail->y) + u + v * block_w);
					val.z = *(reinterpret_cast<int*>(detail->z) + u + v * block_w);

					constexpr float coef_lens_xy = 1.f / 0x80000;
					constexpr float coef_lens_z = 1.f / 0x4000;
					tx.at<float>(v * block_w + u) = val.x * coef_lens_xy;
					ty.at<float>(v * block_w + u) = val.y * coef_lens_xy;
					tz.at<float>(v * block_w + u) = val.z * coef_lens_z;
				}
			}
			delete detail;


			for(int y = 0; y < height; ++y)
			{
				for(int x = 0; x < width; ++x)
				{
					int ref_x[2];
					ref_x[0] = x / 16;
					ref_x[1] = ref_x[0] + 1;
					int ref_y[2];
					ref_y[0] = y / 16;
					ref_y[1] = ref_y[0] + 1;

					float diff_x[2], diff_y[2];
					diff_x[0] = (x - ref_x[0] * 16) / 16.f;
					diff_y[0] = (y - ref_y[0] * 16) / 16.f;
					diff_x[1] = 1.f - diff_x[0];
					diff_y[1] = 1.f - diff_y[0];

					int idx[2][2];
					for(int i = 0; i < 2; ++i)
					{
						for(int j = 0; j < 2; ++j)
						{
							idx[i][j] = (ref_y[i] & (~1)) * ((block_w / 2) << 1)
								+ ((ref_y[i] & 1) << 1) + ((ref_x[j] >> 1) << 2) + (ref_x[j] & 1);
						}
					}

					float ret_x = 0.f;
					ret_x += diff_x[1] * diff_y[1] * tx.at<float>(idx[0][0]);
					ret_x += diff_x[0] * diff_y[1] * tx.at<float>(idx[0][1]);
					ret_x += diff_x[1] * diff_y[0] * tx.at<float>(idx[1][0]);
					ret_x += diff_x[0] * diff_y[0] * tx.at<float>(idx[1][1]);

					float ret_y = 0.f;
					ret_y += diff_x[1] * diff_y[1] * ty.at<float>(idx[0][0]);
					ret_y += diff_x[0] * diff_y[1] * ty.at<float>(idx[0][1]);
					ret_y += diff_x[1] * diff_y[0] * ty.at<float>(idx[1][0]);
					ret_y += diff_x[0] * diff_y[0] * ty.at<float>(idx[1][1]);

					float ret_z = 0.f;
					ret_z += diff_x[1] * diff_y[1] * tz.at<float>(idx[0][0]);
					ret_z += diff_x[0] * diff_y[1] * tz.at<float>(idx[0][1]);
					ret_z += diff_x[1] * diff_y[0] * tz.at<float>(idx[1][0]);
					ret_z += diff_x[0] * diff_y[0] * tz.at<float>(idx[1][1]);

					calib->LensLUT_x.at<float>(y, x) = ret_x;
					calib->LensLUT_y.at<float>(y, x) = ret_y;
					calib->LensLUT_z.at<float>(y, x) = ret_z;
				}
			}

		}
	}


	void init_calibdata(CalibData* calib, const std::string& calibs_files_path)
	{
		cyclic::init(calib, calibs_files_path);
		gradient::init(calib, calibs_files_path);
		temperature::init(calib);
		lens::init(calib, calibs_files_path);
	}


	void init_fusiondata(FusionData* fusion)
	{
		cv::FileStorage fs_TR("TR.xml", cv::FileStorage::READ);
		cv::FileStorage fs_cameraMatrix("rgb_internal.xml", cv::FileStorage::READ);

		fs_TR["T"] >> fusion->t;
		fs_TR["R"] >> fusion->R;
		fs_cameraMatrix["cameraMatrix"] >> fusion->cameraMatrix;

		fusion->t.convertTo(fusion->t, CV_32FC1);
		fusion->R.convertTo(fusion->R, CV_32FC1);
		fusion->cameraMatrix.convertTo(fusion->cameraMatrix, CV_32FC1);
	}

}

