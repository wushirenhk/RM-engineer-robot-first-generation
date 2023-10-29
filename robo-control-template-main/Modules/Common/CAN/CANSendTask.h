#ifndef CANSENDTASK_H
#define CANSENDTASK_H

#include "Task.h"
#include "CanDevice.h"

class Robot;

class CANSendTask : public Task_Base
{
public:
  friend class Robot;
  CANSendTask(Robot &robot0, CanDevice *can_device0, CAN_Tx_Data_Link_t *tx_link0, timeus_t interval_tick_us0 = 0);
  CANSendTask(Robot &robot0, CanDevice *can_device0, uint16_t tx_id, timeus_t interval_tick_us0 = 0);

  virtual ~CANSendTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

protected:
  CAN_Tx_Data_Link_t *can_tx_link;
  CanDevice *can_device;
  CAN_TxHeaderTypeDef can_header;
  uint8_t can_send_data[8];
//protected:
//  CANSendTask *can_send_task_p;
};

#endif