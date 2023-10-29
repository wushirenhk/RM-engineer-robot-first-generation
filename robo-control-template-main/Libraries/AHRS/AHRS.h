#ifndef __AHRS_H__
#define __AHRS_H__

#include "stdint.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix3.h"

class AHRS // 抽象类
{
  friend class MahonyAHRS;
public:
  virtual void init(void) = 0; // 纯虚函数，方法没有定义只有声明
  virtual void update(Vector3f accel, Vector3f gyro) = 0; // 纯虚函数，方法没有定义只有声明
  Quaternion getIMUQuat(void) const; 
  Vector3f getIMUEuler(void) const;
  Matrix3f getIMUCoordinateSystem(void) const;

protected:
  Quaternion imu_quat;
  Vector3f imu_euler;
  Matrix3f imu_coordinate_system;
};

#endif
