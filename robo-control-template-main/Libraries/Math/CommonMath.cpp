/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   CommonMath.cpp
** 文件说明：   数学基本函数
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	   2022-07-10
***************************************************************************/

#include "CommonMath.h"
#include "arm_math.h"
#include "Quaternion.h"
#include "Vector3.h"
#include "Matrix3.h"

namespace cmt
{
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float sq(float x)
{
  return x * x;
}

SimpleAdd<float> simple_adder_instance_f;
PolarAdd<float> polar_adder_instance_f;

}
