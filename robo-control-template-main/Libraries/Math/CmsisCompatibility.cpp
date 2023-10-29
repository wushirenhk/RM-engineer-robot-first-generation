#include "CmsisCompatibility.h"


void compat_arm_quaternion2rotation_f32(Quaternion * _q, Matrix3f * _m)
{
  float q[4] = {_q->w, _q->x, _q->y, _q->z};
  float v[9];
  arm_quaternion2rotation_f32(q, v, 1);

  _m->i.x = v[0];
  _m->i.y = v[3];
  _m->i.z = v[6];

  _m->j.x = v[1];
  _m->j.y = v[4];
  _m->j.z = v[7];

  _m->k.x = v[2];
  _m->k.y = v[5];
  _m->k.z = v[8];

}


void compat_arm_mat_mult_f32(Matrix3f * _m_src1, Matrix3f * _m_src2, Matrix3f * _m_dst)
{
  arm_matrix_instance_f32 src1, src2, dst;
  float src1_array[9] = {_m_src1->i.x, _m_src1->i.y, _m_src1->i.z,
                         _m_src1->j.x, _m_src1->j.y, _m_src1->j.z,
                         _m_src1->k.x, _m_src1->k.y, _m_src1->k.z
                        };
  float src2_array[9] = {_m_src2->i.x, _m_src2->i.y, _m_src2->i.z,
                         _m_src2->j.x, _m_src2->j.y, _m_src2->j.z,
                         _m_src2->k.x, _m_src2->k.y, _m_src2->k.z
                        };
  float dst_array[9];
                        
  arm_mat_init_f32(&src1, 3, 3, src1_array);
  arm_mat_init_f32(&src2, 3, 3, src2_array);
  arm_mat_init_f32(&dst, 3, 3, dst_array);

  arm_mat_mult_f32(&src1, &src2, &dst);

  _m_dst->i.x = dst.pData[0];
  _m_dst->i.y = dst.pData[3];
  _m_dst->i.z = dst.pData[6];
                        
  _m_dst->j.x = dst.pData[1];
  _m_dst->j.y = dst.pData[4];
  _m_dst->j.z = dst.pData[7];
                        
  _m_dst->k.x = dst.pData[2];
  _m_dst->k.y = dst.pData[5];
  _m_dst->k.z = dst.pData[8];

}