#ifndef __LEDCONTROLTASK_H__
#define __LEDCONTROLTASK_H__

#include "Task.h"
#include "stdint.h"
#include "Helper.h"
#include "SongPlayerTask.h"

enum LEDState
{
  LEDSTATE_IMU_INITING = 0,
  LEDSTATE_NORMAL,
  LEDSTATE_OUTOFCONTROL
};

class Robot;

class LEDControlTask : public Task_Base
{
public:
  friend class Robot;
  LEDControlTask(Robot &robot0, timeus_t interval_tick_us0 = 1e6f / 10.0f) : Task_Base(robot0)
  {
    this->interval_tick_us = interval_tick_us0;
  }
  virtual ~LEDControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

  void blink(uint8_t R, uint8_t G, uint8_t B, timeus_t interval_ms, timeus_t dT_us, float duty = 0.5f);

  void breath(uint8_t R, uint8_t G, uint8_t B, timeus_t interval_ms, timeus_t dT_us);

private:
  LEDState state;
  LEDState state_last;

  SongPlayerTask *song_player_task_p;
  //Ìò¹·Ö®¸è
  // uint16_t song_robomasterlickdog[13] = {0x1413, 0x1414, 0x0415, 0x1215, 0x1213, 0x1215, 0x1113, 0x0115, 0x1215, 0x0217, 0x1217, 0x0221, 0x1421};
  // uint16_t sound_warning[3] = {0x4211, 0x4213, 0x4215}; //B__B__B__
  uint16_t mario[7] = {0x1125, 0x8125, 0x8125, 0x1121, 0x1225, 0xF228, 0x4218};
  uint16_t funky_town[10] = {0x1121, 0x1121, 0x111B, 0x8121, 0x8118, 0x1118, 0x1121, 0x1126, 0x1125, 0x1121};
};

#endif
