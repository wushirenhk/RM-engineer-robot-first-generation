#ifndef __INERTIALSENSOR_BACKEND_H__
#define __INERTIALSENSOR_BACKEND_H__

#include "stdint.h"
#include "InertialSensor.h"
#include "Rotation.h"
#include "Filters.h"

// 惯性传感器backend的基类
class InertialSensor_Backend
{
public:
  friend class InertialSensor;
  InertialSensor_Backend(InertialSensor &imu0);
  virtual bool init() = 0;
  virtual bool update() = 0;

protected:
  InertialSensor &imu;
  virtual void updateAccel(void) = 0; // 更新加速度数据，轮询
  virtual void updateGyro(void) = 0;	// 更新陀螺仪数据，轮询
  void correctAccel(void); // 修正加速度数据，轮询
  void correctGyro(void);	// 修正陀螺仪数据，轮询
  void publishAccel(void);
  void publishGyro(void);
  virtual float getTemperature(void) = 0;
  virtual void calibGyro(uint32_t cnt) = 0;

  // 返回imu的id
  uint16_t getId()
  {
    return id;
  }

  uint16_t id; // 后台ID

  Vector3f gyro; // 角速度向量
  Vector3f accel; // 加速度向量
  
  LowPassFilter<Vector3f> accel_lpf;
  LowPassFilter<Vector3f> gyro_lpf;


  Vector3f gyro_offset; // 陀螺仪零漂值
  Vector3f gyro_calib; // 修正后的陀螺仪值
};

#endif
