#ifndef __SONGPLAYERTASK_H__
#define __SONGPLAYERTASK_H__

#include "Task.h"
#include "Scheduler_Common.h"
#include "Helper.h"

class SongPlayerTask : public Task_Base
{
public:
  SongPlayerTask(Robot &robot0, uint16_t * song0, uint16_t song_length0, timeus_t interval_tick_us0 = 1e6f / 40.0f) : Task_Base(robot0)
  {
    this->interval_tick_us = interval_tick_us0;
    self_kill = false;
    song = song0;
    inited = false;
    song_length = song_length0;
  }
  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);
  bool playSong(void);

private:
  bool song_end;
  uint16_t * song;
  uint16_t song_length;

  uint16_t music_steps[4][12] =
  {
    {131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247},
    {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494},
    {523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988},
    {1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976}
  };
  uint16_t bzply_n;
  uint8_t bzply_count;
  uint8_t buzzer_state;
};

#endif
