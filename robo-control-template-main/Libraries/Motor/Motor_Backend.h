#ifndef __MOTOR_BACKEND_H__
#define __MOTOR_BACKEND_H__

#include "stdint.h"
#include "Motor.h"
#include "Motor_Data.h"

class Motor_Backend
{
public:
  friend class Motor;
  friend class Motor_RM;
  Motor_Backend(Motor &motor0);
  virtual void init(void) = 0; // 初始化变量
  virtual bool update(timeus_t dT_us) = 0;
  virtual void uninit(void) = 0; // 初始化变量

  // 获取电机通用测量数据
  Motor_Common_Measurement_t getCommonMeasurement(){
    return com_msr;
  }
  
  // 设置电机线运动基准值
  void setLinearOffset(float offset)
  {
    this->com_msr.linear.offset = offset;
  }
  
//  Motor_Backend(Motor &motor0,
//                uint16_t _id,
//                uint16_t _CAN_Ctrl_Id,
//                uint16_t _CAN_Feedback_Id,
//                float _Reduction_Ratio,
//                int8_t _Direction
//               );
protected:
  Motor &motor;
  virtual void updateMeasurement(void) = 0; // 读取原始数据
  // virtual void processMeasurement(timeus_t dT_us) = 0; // 进一步计算数据
  virtual void publishMeasurement(void);
  uint8_t motor_type;
  uint16_t id;
  uint16_t getId()
  {
    return id;
  }
  Motor_Common_Measurement_t com_msr;

};

#endif