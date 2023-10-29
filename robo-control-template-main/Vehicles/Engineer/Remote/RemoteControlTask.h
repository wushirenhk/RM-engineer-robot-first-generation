#ifdef ENGINEER

#ifndef __REMOTE_CONTROLTASK_H__
#define __REMOTE_CONTROLTASK_H__

#include "Task.h"

class Robot;

class RemoteControlTask : public Task_Base
{
public:

  RemoteControlTask(Robot &robot0, timeus_t interval_tick_us0 = 1e6f / 60.0f) : Task_Base(robot0) // 60 Hz
  {
    this->interval_tick_us = interval_tick_us0;
  }
  float last_vx;
	float last_vy;
	uint8_t open_flag;
	uint8_t close_flag;
  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);
  
   float dm2_delta;
   float dm1_delta;
   int32_t mg1_delta;
   float up_3508_delta;
  
protected:
	uint16_t time_cnt[2];
   uint8_t shift_lock = 0;
   uint16_t shift_tim;
   
   uint8_t press_r_lock = 0;
   uint16_t press_r_tim;

};

#endif

#endif