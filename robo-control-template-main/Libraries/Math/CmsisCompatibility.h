#ifndef CMSISCOMPATIBILITY_H
#define CMSISCOMPATIBILITY_H

#include "arm_math.h"
#include "Quaternion.h"
#include "Matrix3.h"

extern void compat_arm_quaternion2rotation_f32(Quaternion * _q, Matrix3f * _m);
extern void compat_arm_mat_mult_f32(Matrix3f * _m_src1, Matrix3f * _m_src2, Matrix3f * _m_dst);

#endif