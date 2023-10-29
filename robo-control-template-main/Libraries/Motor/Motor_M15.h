#ifndef __MOTOR_M15_H__
#define __MOTOR_M15_H__

#include "Motor_Backend.h"
#include "CanDevice.h"


//DM电机参数
typedef struct _Motor_M15_Params_t
{
	  uint16_t can_tx_id; // CAN总线发送给电机用的ID
     uint16_t can_rx_id; // CAN总线电机发送给开发板用的ID
     uint8_t can_tx_data_start_pos; // CAN总线发送数据起始下标
	  uint8_t canx; // CAN总线标号
	
	  uint16_t set_position;
	  float interval; // 编码器数据解算频率
	
}Motor_M15_Params_t;

typedef struct _Motor_M15_Measurement_t
{
  // 时间戳
  timeus_t timestamp; // 本次时间戳
  
	int16_t given_current;
	int16_t speed_rpm;
   int16_t ecd;
   uint8_t err;
	uint8_t mode;

}Motor_M15_Measurement_t;

class Motor_M15 : public Motor_Backend
{
public:	
  friend class Motor_Backend;
  friend class Arm_ControlTask;

  Motor_M15(Motor &motor0,
           CanDevice &_can_device,
           uint16_t _can_tx_id,
           uint16_t _can_rx_id,
           uint8_t _can_tx_data_start_pos,
           int8_t _motor_type);

  // 通过编码器测量、计算电机角度值
  void updateMotorMeasurement();
  virtual bool update(timeus_t dT_us); // 更新电机输出
  virtual void init(void); // 初始化变量
  virtual void uninit(void); // 反初始化

  virtual void updateMeasurement(void);




protected:

  void syncCANData()
  {
    can_tx_data[0] = set_positon >> 8;
    can_tx_data[1] = set_positon;
  }

  Motor_M15_Measurement_t msr;
  Motor_M15_Params_t params;//参数与测量值
  uint8_t can_tx_data[2];
  uint16_t set_positon;

  uint8_t can_tx_data_start_pos;
  CAN_Rx_Data_Pack_t can_rx_data;
  uint16_t can_tx_id;
  CanDevice &can_device;

};







#endif
