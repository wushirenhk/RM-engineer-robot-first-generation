#ifndef COORDINATE3_H
#define COORDINATE3_H

#include "Vector3.h"

template <typename T>
	
class Matrix3
{
public:
  Vector3<T> i, j, k; // i,j,k为三维矩阵的三个列向量

	// 无参构造三维矩阵
  Matrix3<T>()
  {
    i.set(0, 0, 0);
    j.set(0, 0, 0);
    k.set(0, 0, 0);
  }
	// 通过三维矩阵构造三维矩阵
  Matrix3<T>(const Matrix3<T> &c0)
  {
    i = c0.i;
    j = c0.j;
    k = c0.k;
  }
	// 通过向量构造三维矩阵
  Matrix3<T>(const Vector3<T> i0, const Vector3<T> j0, const Vector3<T> k0)
  {
    i = i0;
    j = j0;
    k = k0;
  }
	// 通过矩阵的每个元素构造
  Matrix3<T>(const T ix, const T jx, const T kx,
             const T iy, const T jy, const T ky,
             const T iz, const T jz, const T kz
            )
  {
    i.set(ix, iy, iz);
    j.set(jx, jy, jz);
    k.set(kx, ky, kz);
  }
	
	// 三维矩阵围绕轴旋转
  Matrix3<T> rotateAboutAxis(Vector3<T> axis, T angle) const
  {
    Matrix3<T> dst;

    T cosine = arm_cos_f32(angle);
    T sine = arm_sin_f32(angle);

    T dot = i.dotProduct(axis);
    Vector3<T> cross = i.crossProduct(axis);

    dst.i.x = cosine * i.x + (1 - cosine) * dot * axis.x - sine * cross.x;
    dst.i.y = cosine * i.y + (1 - cosine) * dot * axis.y - sine * cross.y;
    dst.i.z = cosine * i.z + (1 - cosine) * dot * axis.z - sine * cross.z;


    dot = j.dotProduct(axis);
    cross = j.crossProduct(axis);

    dst.j.x = cosine * j.x + (1 - cosine) * dot * axis.x - sine * cross.x;
    dst.j.y = cosine * j.y + (1 - cosine) * dot * axis.y - sine * cross.y;
    dst.j.z = cosine * j.z + (1 - cosine) * dot * axis.z - sine * cross.z;


    dot = k.dotProduct(axis);
    cross = k.crossProduct(axis);

    dst.k.x = cosine * k.x + (1 - cosine) * dot * axis.x - sine * cross.x;
    dst.k.y = cosine * k.y + (1 - cosine) * dot * axis.y - sine * cross.y;
    dst.k.z = cosine * k.z + (1 - cosine) * dot * axis.z - sine * cross.z;

    return dst;
  }

  // 获取行向量，'x','y','z'
  Vector3<T> getRowVector(uint8_t row)
  {
    Vector3<T> dst;
    switch(row)
    {
    case 'x':
      dst.x = i.x;
      dst.y = j.x;
      dst.z = k.x;
      break;
    case 'y':
      dst.x = i.y;
      dst.y = j.y;
      dst.z = k.y;
      break;
    case 'z':
      dst.x = i.z;
      dst.y = j.z;
      dst.z = k.z;
      break;
    default:
      break;
    }
    return dst;
  }
	// 三维矩阵右乘向量，转换为向量
  Vector3<T> translateVector(const Vector3<T> & vec) const
  {
    Vector3<T> dst;
    dst.x = vec.dotProduct(getRowVector('x'));
    dst.y = vec.dotProduct(getRowVector('y'));
    dst.z = vec.dotProduct(getRowVector('z'));
    return dst;
  }
  // 右乘三维矩阵
  Matrix3<T> translateMatrix(const Matrix3<T> & mat) const
  {
    Matrix3<T> dst;
    dst.i = *this->translateVector(mat.i);
    dst.j = *this->translateVector(mat.j);
    dst.k = *this->translateVector(mat.k);
    return dst;
  }
  // 标准化
  void normalize()
  {
    this->i.normalize();
    this->j.normalize();    
    this->k.normalize();
  }

};

typedef Matrix3<int16_t>                Matrix3i;
typedef Matrix3<uint16_t>               Matrix3ui;
typedef Matrix3<int32_t>                Matrix3l;
typedef Matrix3<uint32_t>               Matrix3ul;
typedef Matrix3<float>                  Matrix3f;
typedef Matrix3<double>                 Matrix3d;

#endif