#ifndef __VECTOR2_H__
#define __VECTOR2_H__

#include "stdint.h"
#include "math.h"
#include "CommonMath.h"

template <typename T>
class Vector2
{
public:
  T x, y;

  // 初始化向量为零向量
  Vector2<T>()
  {
    x = 0;
    y = 0;
  }

  // 通过向量构造
  Vector2<T>(const Vector2<T> &v0)
  {
    x = v0.x;
    y = v0.y;
  }

  // 通过坐标构造
  Vector2<T>(const T x0, const T y0)
  {
    x = x0;
    y = y0;
  }
  
  // 通过坐标设定
  void set(const T x0, const T y0)
  {
    x = x0;
    y = y0;
  }
  
  // 通过向量设定
  void set(const Vector2<T> &v)
  {
    x = v.x;
    y = v.y;
  }

  // 判断是否相等
  bool isEqual(const Vector2<T> &v) const
  {
    return ((x == v.x) && (y == v.y));
  }

  // 判断是否不等
  bool isNotEqual(const Vector2<T> &v) const
  {
    return (!(x == v.x) || !(y == v.y));
  }

  // 负号
  Vector2<T> negation(void) const
  {
    return Vector2<T>(-x, -y);
  }

  // 加
  Vector2<T> add(const Vector2<T> &v) const
  {
    return Vector2<T>(x + v.x, y + v.y);
  }

  // 减
  Vector2<T> subtract(const Vector2<T> &v) const
  {
    return Vector2<T>(x - v.x, y - v.y);
  }

  // 等比缩放
  Vector2<T> uniformScaling(const T num) const
  {
    return Vector2<T>(x * num, y * num);
  }

  // 不等比缩放
  Vector2<T> nonUniformScaling(const Vector2<T> &v)
  {
    return Vector2<T>(x * v.x, y * v.y);
  }

  // 不等比缩放
  Vector2<T> nonUniformScaling(const T &x0, const T &y0)
  {
    return Vector2<T>(x * x0, y * y0);
  }

  // 点乘
  T dotProduct(const Vector2<T> &v) const
  {
    return x * v.x + y * v.y;
  }
  
  // 叉乘
  T crossProduct(const Vector2<T> &v) const
  {
    return x * v.y - y * v.x;
  }

  // 计算与向量v的夹角[0, PI]
  T angle(const Vector2<T> &v) const
  {
    const T len = this->length() * v.length();
    if (len <= 0)
    {
      return 0.0f;
    }
    const T cosv = (dotProduct(v)) / len;
    if (cosv > 1 || cosv < -1)
    {
      return 0.0f;
    }
    return acosf(cosv);
  }

  // 计算转向向量v所需的最小转角[-PI, PI]
  T anglePolar(const Vector2<T> &v) const
  {
    return atan2f(crossProduct(v), dotProduct(v));
  }
  
  // 检查是否有一个分量为NaN
  bool isNaN(void) const
  {
    return isnan(x) || isnan(y);
  }

  // 检查是否有一个分量为Inf
  bool isInf(void) const
  {
    return isinf(x) || isinf(y);
  }

  // 检查是否为零向量
  bool isZero(void) const
  {
    return cmt::isZero(x) && cmt::isZero(y);
  }

  // 将向量设为零向量
  void zero(void)
  {
    x = y = 0;
  }

  // 获得向量的模长
  T length(void) const
  {
    return sqrtf(x * x + y * y);
  }
  
  // 获得向量的模长平方
  T length2(void) const
  {
    return x * x + y * y;
  }

  // 标准化
  void normalize(void)
  {
    T recip_norm = (T) cmt::invSqrt(x * x + y * y);
    x *= recip_norm;
    y *= recip_norm;
  }
  
  // 返回标准化后的向量
  Vector2<T> normalized(void)
  {
    T recip_norm = (T) cmt::invSqrt(x * x + y * y);
    Vector2<T> v = *this;
    v.x *= recip_norm;
    v.y *= recip_norm;
    return v;
  }

  // 点乘运算符*
  T operator *(const Vector2<T> &v) const
  {
    return x * v.x + y * v.y;
  }

  // 等比缩放赋值运算符*=
  Vector2<T> &operator *=(const T num)
  {
    x *= num;
    y *= num;
    return *this;
  }

  // 等比缩放赋值运算符/=
  Vector2<T> &operator /=(const T num)
  {
    x /= num;
    y /= num;
    return *this;
  }

  // 减法赋值运算符-=
  Vector2<T> &operator -=(const Vector2<T> &v)
  {
    x -= v.x;
    y -= v.y;
    return *this;
  }

  // 加法赋值运算符+=
  Vector2<T> &operator +=(const Vector2<T> &v)
  {
    x += v.x;
    y += v.y;
    return *this;
  }

  // 等比缩放运算符/
  Vector2<T> operator /(const T num) const
  {
    return Vector2<T>(x / num, y / num);
  }

  // 等比缩放运算符*
  Vector2<T> operator *(const T num) const
  {
    return Vector2<T>(x * num, y * num);
  }

  // 减法运算符-
  Vector2<T> operator -(const Vector2<T> &v) const
  {
    return Vector2<T>(x - v.x, y - v.y);
  }

  // 加法运算符+
  Vector2<T> operator +(const Vector2<T> &v) const
  {
    return Vector2<T>(x + v.x, y + v.y);
  }

  // 反向运算符-
  Vector2<T> operator -(void) const
  {
    return Vector2<T>(-x, -y);
  }

  // 判断是否相等运算符==
  bool operator ==(const Vector2<T> &v) const
  {
    return ((x == v.x) && (y == v.y));
  }

  // 判断是否不等运算符！=
  bool operator !=(const Vector2<T> &v) const
  {
    return (!(x == v.x) || !(y == v.y));
  }

};

typedef Vector2<int16_t>                Vector2i;
typedef Vector2<uint16_t>               Vector2ui;
typedef Vector2<int32_t>                Vector2l;
typedef Vector2<uint32_t>               Vector2ul;
typedef Vector2<double>                 Vector2d;
typedef Vector2<float>                  Vector2f;


#endif
