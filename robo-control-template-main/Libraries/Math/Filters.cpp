#include "Filters.h"
#include "arm_math.h"
#include "Vector3.h"


/***********************************************************************
** 函 数 名： LowPassFilter<Vector3f>::LowPassFilter
** 函数说明： 有参构造函数，完成初始化
**---------------------------------------------------------------------
** 输入参数： _dT_us：采样频率；_cut_off_freq：截止频率
** 返回参数： 
***********************************************************************/
template<>
LowPassFilter<Vector3f>::LowPassFilter(float _dT_us, float _cut_off_freq)
{

  dT_us = _dT_us;
  cut_off_freq = _cut_off_freq;

  updateCoef();

  xn.zero();
  yn.zero();
  yn_1.zero();

}


/***********************************************************************
** 函 数 名： LowPassFilter<float>::LowPassFilter
** 函数说明： 有参构造函数，完成初始化
**---------------------------------------------------------------------
** 输入参数： _dT_us：采样频率；_cut_off_freq：截止频率
** 返回参数： 
***********************************************************************/
template<>
LowPassFilter<float>::LowPassFilter(float _dT_us, float _cut_off_freq)
{

  dT_us = _dT_us;
  cut_off_freq = _cut_off_freq;

  updateCoef();

  xn = 0;
  yn = 0;
  yn_1 = 0;

}

/***********************************************************************
** 函 数 名： LowPassFilter<Vector3f>::update
** 函数说明： 数字低通滤波处理函数
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
template<>
void LowPassFilter<Vector3f>::update(void)
{
  yn =  (yn_1) * (1 - coef) + (xn) * (coef);
  yn_1 = yn;
}

/***********************************************************************
** 函 数 名： LowPassFilter<float>::update
** 函数说明： 数字低通滤波处理函数
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
template<>
void LowPassFilter<float>::update(void)
{
  yn = (1 - coef) * (yn_1) + (coef) * (xn);
  yn_1 = yn;
}
