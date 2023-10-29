#ifndef MOTOR_M15_TASK_H
#define MOTOR_M15_TASK_H

#include "Task.h"
#include "Motor_M15.h"
#include "CANSendTask.h"


class Motor_M15_PoControlTask : public Task_Base
{

public:
	  friend class Arm_ControlTask;

  Motor_M15_PoControlTask(
    Robot &robot0,
    CanDevice &_can_device,
    uint16_t _can_rx_id, uint16_t _can_tx_id,
    uint8_t _can_tx_data_start_pos,
    int8_t _motor_type,
    timeus_t interval_tick_us0 = 0
  );

  virtual ~Motor_M15_PoControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);



protected:
	
  Motor_M15 *motor_backend_p;
  CanDevice &can_device;
  uint8_t can_tx_data_start_pos;

};



#endif