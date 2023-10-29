#ifndef MOTOR_MG_TASKS_H
#define MOTOR_MG_TASKS_H

#include "Task.h"
#include "stdint.h"
#include "Motor_MG.h"
#include "CANSendTask.h"

class Robot;

class Motor_MG_ControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class Arm_ControlTask;
  friend class RemoteControlTask;
  friend class Motor_MG;

  Motor_MG_ControlTask(
    Robot &robot0,
    CanDevice &_can_device,
    uint16_t _can_rx_id, uint16_t _can_tx_id,
    uint8_t _can_tx_data_start_pos,
    int8_t _motor_type,
    timeus_t interval_tick_us0 = 0

  );

  virtual ~Motor_MG_ControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

  void setHomePosition();//电机上电复位
		
protected:
   Motor_MG *motor_backend_p;
   CanDevice &can_device;
   uint8_t can_tx_data_start_pos;
  // CANSendTask *can_send_task_p;
  // CAN_Rx_Data_Pack_t can_rx_data;

};


#endif
