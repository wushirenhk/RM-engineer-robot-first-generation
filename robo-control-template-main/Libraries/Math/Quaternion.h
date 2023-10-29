#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "stdint.h"
#include "math.h"
#include "CommonMath.h"
#include "Matrix3.h"

template <typename T> // 模板头，template关键字告诉编译器开始泛型编程
  
/* 包含四元数类的构造函数，各种运算方法 */
struct QuaternionT // 模板结构体
{

  T w, x, y, z;

  QuaternionT<T>() // 无参构造函数
  {
    w = 1;
    x = 0;
    y = 0;
    z = 0;
  }

  QuaternionT<T>(const QuaternionT<T> &q0) // 拷贝构造函数
  {
    w = q0.w;
    x = q0.x;
    y = q0.y;
    z = q0.z;
  }

  QuaternionT<T>(const T w0, const T x0, const T y0, const T z0) // 构造函数
  {
    w = w0;
    x = x0;
    y = y0;
    z = z0;
  }

  void set(const T w0, const T x0, const T y0, const T z0) // 四元数赋值
  {
    w = w0;
    x = x0;
    y = y0;
    z = z0;
  }

  void set(const QuaternionT<T> &v0) // 拷贝赋值
  {
    w = v0.w;
    x = v0.x;
    y = v0.y;
    z = v0.z;
  }

  // test for equality
  bool isEqual(const QuaternionT<T> &v0) const
  {
    return ((x == v0.x) && (y == v0.y) && (z == v0.z) && (w == v0.w));
  }

  // test for inequality
  bool isNotEqual(const QuaternionT<T> &v0) const
  {
    return (!(x == v0.x) || !(y == v0.y) || !(z == v0.z) || !(w == v0.w));
  }

  // negation
  QuaternionT<T> negation(void) const
  {
    return QuaternionT<T>(-w, -x, -y, -z);
  }

  // add加
  QuaternionT<T> add(const QuaternionT<T> &v0) const
  {
    return QuaternionT<T>(w + v0.w, x + v0.x, y + v0.y, z + v0.z);
  }

  // subtract减
  QuaternionT<T> subtract(const QuaternionT<T> &v0) const
  {
    return QuaternionT<T>(w - v0.w, x - v0.x, y - v0.y, z - v0.z);
  }

  // uniform scaling（等比缩放）
  QuaternionT<T> uniformScaling(const T num) const
  {
    return QuaternionT<T>(w * num, x * num, y * num, z * num);
  }

  // non-uniform scaling（不等比缩放）
  QuaternionT<T> nonUniformScaling(const QuaternionT<T> &q0) const
  {
    w *= q0.w;
    x *= q0.x;
    y *= q0.y;
    z *= q0.z;
  }

  // check if any elements are NAN
  bool isNaN(void) const
  {
    return isnan(w) || isnan(x) || isnan(y) || isnan(z);
  }

  // check if any elements are infinity（无穷大）
  bool isInf(void) const
  {
    return isinf(w) || isinf(x) || isinf(y) || isinf(z);
  }
  
  // Checks if each element of the quaternion is zero
  bool isZero(void) const
  {
    return cmt::isZero(w) && cmt::isZero(x) && cmt::isZero(y) && cmt::isZero(z);
  }

	// 求四元数的模
  T length(void) const
  {
    return sqrtf(w * w + x * x + y * y + z * z);
  }

  // gets the length squared of the quaternion
  T lengthSquared() const
  {
    return (T)(w * w + x * x + y * y + z * z);
  }

  void normalize(void) // 标准化向量
  {
    T recip_norm = (T)cmt::invSqrt(w * w + x * x + y * y + z * z);
    w *= recip_norm;
    x *= recip_norm;
    y *= recip_norm;
    z *= recip_norm;

  };

  QuaternionT<T> product(const QuaternionT<T> & q0) const // 四元数相乘
  {
    QuaternionT<T> dst;
    dst.x =  x * q0.w + y * q0.z - z * q0.y + w * q0.x;
    dst.y = -x * q0.z + y * q0.w + z * q0.x + w * q0.y;
    dst.z =  x * q0.y - y * q0.x + z * q0.w + w * q0.z;
    dst.w = -x * q0.x - y * q0.y - z * q0.z + w * q0.w;
    return dst;
  }

  QuaternionT<T> imag(void) const // 取四元数的虚部
  {
    QuaternionT<T> dst;
    dst.x = x;
    dst.y = y;
    dst.z = z;
    dst.w = 0;
    return dst;
  }
  
  QuaternionT<T> conjugate(void) const // 取四元数的共轭
  {
    QuaternionT<T> dst;
    dst.x = -x;
    dst.y = -y;
    dst.z = -z;
    dst.w =  w;
    return dst;
  }
  
  QuaternionT<T> inverse(void) const // 求四元数的逆
  {
    QuaternionT<T> dst;
    T scale;
    dst = dst.conjugate();
    scale = *this->lengthSquared();
    dst.uniformScaling(1.0f/scale);
    return dst;
  }
  
  Matrix3<T> toRotationMatrix(void) const // 四元数转旋转矩阵
  {
    // The quaternion a + ib + jc + kd is converted into rotation matrix:
    // <pre>
    //   w^2 + x^2 - y^2 - z^2                 2xy - 2wz                 2xz + 2wy
    //               2xy + 2wz     w^2 - x^2 + y^2 - z^2                 2yz - 2wx
    //               2xz - 2wy                 2yz + 2wx     w^2 - x^2 - y^2 + z^2
    // </pre>
    
    Matrix3<T> dst;
    dst.i.x = w * w + x * x - y * y - z * z;
    dst.i.y = 2 * x * y + 2 * w * z;
    dst.i.z = 2 * x * z - 2 * w * y;
    
    dst.j.x = 2 * x * y - 2 * w * z;
    dst.j.y = w * w - x * x + y * y - z * z;
    dst.j.z = 2 * y * z + 2 * w * x;
    
    dst.k.x = 2 * x * z + 2 * w * y;
    dst.k.y = 2 * y * z - 2 * w * x;
    dst.k.z = w * w - x * x - y * y + z * z;
    
    return dst;
  }
};

typedef QuaternionT<float> Quaternion;
typedef QuaternionT<double> QuaternionD;


#endif
