#ifndef __COMMONMATH_H__
#define __COMMONMATH_H__

#include "stdint.h"
#include "math.h"
#include "float.h"
#include "arm_math.h"

// 安全除法
#define safeDiv(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )

// 返回限制最大值后的数值
#define limitMax(x, max) (x > max ? max : x)

// 返回限制最小值后的数值
#define limitMin(x, min) (x < min ? min : x)

// 返回限制了最大和最小值后的数值
#define limit(x, min, max) ((x > max ? max : x) < min ? min : (x > max ? max : x))

//

namespace cmt
{

  
// 判断是否为0
inline bool isZero(const float x)
{
  return (x < FLT_EPSILON && x > - FLT_EPSILON);
}


float invSqrt(float x);
float sq(float x);
// 高级的加法
template<class T>
class AdvancedAdd_Base
{
public:
  AdvancedAdd_Base() {}
  virtual T operator() (T a, T b) = 0;
};

template<class T>
  class SimpleAdd : public AdvancedAdd_Base<T>
{
public:
  SimpleAdd() {}
  virtual T operator() (T a, T b)
  {
    return a + b;
  }
};

// 极坐标用角度加减法 (-PI, PI]
template<class T>
class PolarAdd : public AdvancedAdd_Base<T>
{
public:
  PolarAdd()
  {
    this->min = -PI;
    this->max = PI;
  }
  virtual T operator() (T a, T b)
  {
    T c = a + b;
    len = max - min;
//    c -= (int32_t)(c/len) * len;
//    c += min;
    while(c <= min) c += len;
    while(c > max) c -= len;
    return c;
  }
protected:
  T min, max, len;
};


extern SimpleAdd<float> simple_adder_instance_f;
extern PolarAdd<float> polar_adder_instance_f;


}

#endif
