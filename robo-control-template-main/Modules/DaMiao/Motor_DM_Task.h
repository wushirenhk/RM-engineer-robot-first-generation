#ifndef MOTOR_DM_TASK_H
#define MOTOR_DM_TASK_H

#include "Task.h"
#include "Motor_DM.h"
#include "CANSendTask.h"

class Motor_DM_PSControlTask : public Task_Base
{

public:
	  friend class Arm_ControlTask;
    friend class RemoteControlTask;
     
  Motor_DM_PSControlTask(
    Robot &robot0,
    CanDevice &_can_device,
    uint16_t _can_rx_id, uint16_t _can_tx_id,
    uint8_t _can_tx_data_start_pos,
    int8_t _motor_type,
    timeus_t interval_tick_us0 = 0
  );

  virtual ~Motor_DM_PSControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);


protected:
	
  Motor_DM *motor_backend_p;
  CanDevice &can_device;
  uint8_t can_tx_data_start_pos;
  



};


#endif