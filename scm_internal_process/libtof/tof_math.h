#ifndef CIS_TOF_MATH_HEADER
#define CIS_TOF_MATH_HEADER

#include <arm_neon.h>
#include <numbers>

namespace cis
{

inline float32x4_t v_pairwise_poly_3(float32x4_t x, float32x4_t x2, float32x4_t* poly)
{
	float32x4_t p01 = vfmaq_f32(poly[0], x, poly[1]);
	float32x4_t p23 = vfmaq_f32(poly[2], x, poly[3]);
	return vfmaq_f32(p01, x2, p23);
}

inline float32x4_t vatan2q_f32(float32x4_t y, float32x4_t x)
{
	float32x4_t poly[8];

	poly[0] = vdupq_n_f32(-0x1.55555p-2f);
	poly[1] = vdupq_n_f32(0x1.99935ep-3f);
	poly[2] = vdupq_n_f32(-0x1.24051ep-3f);
	poly[3] = vdupq_n_f32(0x1.bd7368p-4f);
	poly[4] = vdupq_n_f32(-0x1.491f0ep-4f);
	poly[5] = vdupq_n_f32(0x1.93a2c0p-5f);
	poly[6] = vdupq_n_f32(-0x1.4c3c60p-6f);
	poly[7] = vdupq_n_f32(0x1.01fd88p-8f);

	uint32x4_t ix = vreinterpretq_u32_f32(x);
	uint32x4_t iy = vreinterpretq_u32_f32(y);

	uint32x4_t sign_mask = vdupq_n_u32(0x80000000);
	uint32x4_t sign_x = vandq_u32(ix, sign_mask);
	uint32x4_t sign_y = vandq_u32(iy, sign_mask);
	uint32x4_t sign_xy = veorq_u32(sign_x, sign_y);

	float32x4_t ax = vabsq_f32(x);
	float32x4_t ay = vabsq_f32(y);

	uint32x4_t pred_xlt0 = vcltzq_f32(x);
	uint32x4_t pred_aygtax = vcgtq_f32(ay, ax);

	float32x4_t n = vbslq_f32(pred_aygtax, vnegq_f32(ax), ay);
	float32x4_t d = vbslq_f32(pred_aygtax, ay, ax);
	float32x4_t z = vdivq_f32(n, d);

	float32x4_t shift = vreinterpretq_f32_u32(
			vandq_u32(pred_xlt0, vreinterpretq_u32_f32(vdupq_n_f32(-2.f))));
	shift = vbslq_f32(pred_aygtax, vaddq_f32(shift, vdupq_n_f32(1.f)), shift);
	shift = vmulq_f32(shift, vdupq_n_f32(0x1.921fb6p+0f));

	float32x4_t z2 = vmulq_f32(z, z);
	float32x4_t z4 = vmulq_f32(z2, z2);

	float32x4_t ret = vfmaq_f32(v_pairwise_poly_3(z2, z4, &poly[0]), z4,
			  vmulq_f32(z4, v_pairwise_poly_3(z2, z4, &poly[4])));
	ret = vaddq_f32(vfmaq_f32(z, ret, vmulq_f32(z2, z)), shift);
	ret = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(ret), sign_xy));

	return ret;
}

inline float32x4_t fmodf_2pi(float32x4_t x)
{
	constexpr float pi = std::numbers::pi_v<float>;
	float32x4_t y = vdupq_n_f32(2.f * pi);
	float32x4_t div_xy = vrndq_f32(vdivq_f32(x, y));
	return vfmsq_f32(x, div_xy, y);
}

} /* namespace cis */

#endif /* CIS_TOF_MATH_HEADER */
