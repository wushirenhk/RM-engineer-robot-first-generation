#ifndef __VECTOR3_H__
#define __VECTOR3_H__

#include "stdint.h"
#include "math.h"
#include "CommonMath.h"

template <typename T>
class Vector3
{
public:
  T x, y, z;

  // 初始化向量为零向量
  Vector3<T>()
  {
    x = 0;
    y = 0;
    z = 0;
  }

  // 通过向量构造
  Vector3<T>(const Vector3<T> &v0)
  {
    x = v0.x;
    y = v0.y;
    z = v0.z;
  }

  // 通过坐标构造
  Vector3<T>(const T x0, const T y0, const T z0)
  {
    x = x0;
    y = y0;
    z = z0;
  }
  
  // 通过坐标设定
  void set(const T x0, const T y0, const T z0)
  {
    x = x0;
    y = y0;
    z = z0;
  }
  
  // 通过向量设定
  void set(const Vector3<T> &v)
  {
    x = v.x;
    y = v.y;
    z = v.z;
  }

  // 判断是否相等
  bool isEqual(const Vector3<T> &v) const
  {
    return ((x == v.x) && (y == v.y) && (z == v.z));
  }

  // 判断是否不等
  bool isNotEqual(const Vector3<T> &v) const
  {
    return (!(x == v.x) || !(y == v.y) || !(z == v.z));
  }

  // 负号
  Vector3<T> negation(void) const
  {
    return Vector3<T>(-x, -y, -z);
  }

  // 加
  Vector3<T> add(const Vector3<T> &v) const
  {
    return Vector3<T>(x + v.x, y + v.y, z + v.z);
  }

  // 减
  Vector3<T> subtract(const Vector3<T> &v) const
  {
    return Vector3<T>(x - v.x, y - v.y, z - v.z);
  }

  // 等比缩放
  Vector3<T> uniformScaling(const T num) const
  {
    return Vector3<T>(x * num, y * num, z * num);
  }

  // 不等比缩放
  Vector3<T> nonUniformScaling(const Vector3<T> &v)
  {
    return Vector3<T>(x * v.x, y * v.y, z * v.z);
  }

  // 不等比缩放
  Vector3<T> nonUniformScaling(const T &x0, const T &y0, const T &z0)
  {
    return Vector3<T>(x * x0, y * y0, z * z0);
  }

  // 点乘
  T dotProduct(const Vector3<T> &v) const
  {
    return x * v.x + y * v.y + z * v.z;
  }

  // 叉乘
  Vector3<T> crossProduct(const Vector3<T> &v) const
  {
    Vector3<T> temp(y * v.z - z * v.y,
                    z * v.x - x * v.z,
                    x * v.y - y * v.x);
    return temp;
  }

  // 混合积 det([*this, v, v1])
  T mixedProduct(const Vector3<T> &v, const Vector3<T> &v1) const
  {
    Vector3<T> cross = crossProduct(v);
    return cross.dotProduct(v1);
  }

  // 计算两向量夹角
  T angle(const Vector3<T> &v) const
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

  // 检查是否有一个分量为NaN
  bool isNaN(void) const
  {
    return isnan(x) || isnan(y) || isnan(z);
  }

  // 检查是否有一个分量为Inf
  bool isInf(void) const
  {
    return isinf(x) || isinf(y) || isinf(z);
  }

  // 检查是否为零向量
  bool isZero(void) const
  {
    return cmt::isZero(x) && cmt::isZero(y) && cmt::isZero(z);
  }

  // 将向量设为零向量
  void zero(void)
  {
    x = y = z = 0;
  }

  // 获得向量的模长
  T length(void) const
  {
    return sqrtf(x * x + y * y + z * z);
  }
  
  // 获得向量的模长平方
  T length2(void) const
  {
    return x * x + y * y + z * z;
  }

  // 标准化
  void normalize(void)
  {
    T recip_norm = (T) cmt::invSqrt(x * x + y * y + z * z);
    x *= recip_norm;
    y *= recip_norm;
    z *= recip_norm;
  }
  
  // 返回标准化后的向量
  Vector3<T> normalized(void)
  {
    T recip_norm = (T) cmt::invSqrt(x * x + y * y + z * z);
    Vector3<T> v = *this;
    v.x *= recip_norm;
    v.y *= recip_norm;
    v.z *= recip_norm;
    return v;
  }

  // 归一化的向量绕归一化的轴旋转
  Vector3<T> rotateAboutAxis(Vector3<T> axis, T angle) const
  {
    T cosine = arm_cos_f32(angle);
    T sine = arm_sin_f32(angle);

    T dot = dotProduct(axis);
    Vector3<T> cross = crossProduct(axis);

    Vector3<T> dst;
    dst.x = cosine * x + (1 - cosine) * dot * axis.x - sine * cross.x;
    dst.y = cosine * y + (1 - cosine) * dot * axis.y - sine * cross.y;
    dst.z = cosine * z + (1 - cosine) * dot * axis.z - sine * cross.z;

    return dst;
  }

  // 叉乘运算符%
  Vector3<T> operator %(const Vector3<T> &v) const
  {
    Vector3<T> temp(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    return temp;
  }
  
  // 点乘运算符*
  T operator *(const Vector3<T> &v) const
  {
    return x * v.x + y * v.y + z * v.z;
  }

  // 等比缩放赋值运算符*=
  Vector3<T> &operator *=(const T num)
  {
    x *= num;
    y *= num;
    z *= num;
    return *this;
  }

  // 等比缩放赋值运算符/=
  Vector3<T> &operator /=(const T num)
  {
    x /= num;
    y /= num;
    z /= num;
    return *this;
  }

  // 减法赋值运算符-=
  Vector3<T> &operator -=(const Vector3<T> &v)
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  // 加法赋值运算符+=
  Vector3<T> &operator +=(const Vector3<T> &v)
  {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }

  // 等比缩放运算符/
  Vector3<T> operator /(const T num) const
  {
    return Vector3<T>(x / num, y / num, z / num);
  }

  // 等比缩放运算符*
  Vector3<T> operator *(const T num) const
  {
    return Vector3<T>(x * num, y * num, z * num);
  }

  // 减法运算符-
  Vector3<T> operator -(const Vector3<T> &v) const
  {
    return Vector3<T>(x - v.x, y - v.y, z - v.z);
  }

  // 加法运算符+
  Vector3<T> operator +(const Vector3<T> &v) const
  {
    return Vector3<T>(x + v.x, y + v.y, z + v.z);
  }

  // 反向运算符-
  Vector3<T> operator -(void) const
  {
    return Vector3<T>(-x, -y, -z);
  }

  // 判断是否相等运算符==
  bool operator ==(const Vector3<T> &v) const
  {
    return ((x == v.x) && (y == v.y) && (z == v.z));
  }

  // 判断是否不等运算符！=
  bool operator !=(const Vector3<T> &v) const
  {
    return (!(x == v.x) || !(y == v.y) || !(z == v.z));
  }

};

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<double>                 Vector3d;
typedef Vector3<float>                  Vector3f;


#endif
