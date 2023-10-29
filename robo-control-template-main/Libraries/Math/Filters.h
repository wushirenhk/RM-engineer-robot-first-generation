#ifndef _FILTERS_H_
#define _FILTERS_H_

#include "stdint.h"
#include "CommonMath.h"

// 低通滤波
template<class T> // 模板头，template关键字告诉编译器开始泛型编程
class LowPassFilter
{
public:
  LowPassFilter<T>() {}
  LowPassFilter<T>(float _dT_us, float _cut_off_freq);
	
	// 一阶数字低通滤波处理
  T calc(float _dT_us, T _xn)
  {
    dT_us = _dT_us;
    if(cmt::isZero(dT_us))
      return yn * 0;

    xn = _xn;
		
    updateCoef(); // 设置滤波系数
    limitCoef(); // 限制coef大小
		
    update(); // 数字低通滤波处理

    return yn;
  }
	
	// 调出滤波结果yn
  T getOutput(void)
  {
    return yn;
  }
	
	// 设置采样频率
  void setSamplingCycle(float t)
  {
    dT_us = t;
  }
	
	// 设置滤波截至频率
  void setCutOffFreq(float freq)
  {
    cut_off_freq = freq;
  }

protected:
  T xn; // 第n次采样的数据
  T yn; // 第n次滤波器输出值
  T yn_1; // 第n-1次滤波器输出值
  float coef; // 滤波系数
  float cut_off_freq; // 滤波器截止频率
  float dT_us; // 采样周期（us）

	// 限制coef大小
  void limitCoef()
  {
    coef = limit(coef, 0.0f, 1.0f);
  }
	
	// 更新数字滤波系数，取决于截止频率
  void updateCoef()
  {
    coef = cut_off_freq * 2 * PI * dT_us / 1e6f; // 采样系数=采样周期/时间常数=采样周期 * 截止角频率
  }
  void update();
};


#endif
