/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   LEDControlTask.cpp
** 文件说明：   LED控制任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	     2022-07-18
**	 1.1							   补充注释						     赵钟源     	     2022-12-10
***************************************************************************/

#include "LEDControlTask.h"


#include "/Robot/Robot.h"


/***********************************************************************
** 函 数 名： LEDControlTask::init()
** 函数说明： LED任务初始化（设定LED初始状态为IMU初始化）
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void LEDControlTask::init(void)
{
  inited = true;
  state = LEDSTATE_IMU_INITING;
  state_last = state;
}

/***********************************************************************
** 函 数 名： LEDControlTask::update(timeus_t dT_us)
** 函数说明： LED任务更新，呼吸灯
**---------------------------------------------------------------------
** 输入参数： 更新时间周期（us）
** 返回参数： 无
***********************************************************************/
void LEDControlTask::update(timeus_t dT_us)
{
  static bool song_end = false;
  state_last = state;

  // 旧活新整
//	if(++timer == 5 && !song_end)
//	{
//		song_end = robot.helper.playingSong(song_robomasterlickdog, 13);
//		timer = 0;
//	}
  if(!isOutOfClock(robot.rc_protocol.getRCData().timestamp_us))
  {
    robot.rc_protocol.setValid(true);
  }
  else
  {
    robot.rc_protocol.setValid(false);
  }
  if(state == LEDSTATE_IMU_INITING && robot.inertial_sensors.isInited())
  {
    state = LEDSTATE_NORMAL;
  }
  if(state == LEDSTATE_NORMAL && !robot.rc_protocol.isValid())
  {
    state = LEDSTATE_OUTOFCONTROL;
  }
  if(state == LEDSTATE_OUTOFCONTROL && robot.rc_protocol.isValid())
  {
    state = LEDSTATE_NORMAL;
  }
  if(state == LEDSTATE_IMU_INITING)
  {
    blink(1, 1, 1, 200, dT_us);
  }
  if(state == LEDSTATE_NORMAL)
  {
    // blink(0, 0, 1, 600, dT_us);
    breath(0, 0, 1, 600, dT_us);
  }
  if(state == LEDSTATE_OUTOFCONTROL)
  {
    breath(1, 1, 0, 600, dT_us);
  }
  if((state_last == LEDSTATE_OUTOFCONTROL || state_last == LEDSTATE_IMU_INITING) && state == LEDSTATE_NORMAL)
  {
    song_player_task_p = new SongPlayerTask(robot, mario, 7, 1800); // 18000
    robot.scheduler.registerTask(song_player_task_p);
    //robot.scheduler.unregisterTask((Task_Base**)&(robot.referee_system_task_p));
  }
  if(state != state_last)
  {
    robot.helper.setLED(0, 0, 0);
  }
}

/***********************************************************************
** 函 数 名： LEDControlTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void LEDControlTask::uninit(void)
{

}

/***********************************************************************
** 函 数 名： LEDControlTask::blink(uint8_t R, uint8_t G, uint8_t B,
                                    timeus_t interval_ms, timeus_t dT_us,
                                    float duty)
** 函数说明： LED闪灯
**---------------------------------------------------------------------
** 输入参数： 颜色（RGB），闪灯间隔，更新间隔、占空比
** 返回参数： 无
***********************************************************************/
void LEDControlTask::blink(uint8_t R, uint8_t G, uint8_t B, timeus_t interval_ms, timeus_t dT_us, float duty)
{
  static timeus_t timer_us = 0;
  if(timer_us >= interval_ms * 1000 * duty)
  {
    robot.helper.setLED(0, 0, 0);
  }
  if(timer_us >= interval_ms * 1000 && duty > .05f)
  {
    robot.helper.setLED(R, G, B);
    timer_us = 0;
  }
  timer_us += dT_us;
}

/***********************************************************************
** 函 数 名： LEDControlTask::breath(uint8_t R, uint8_t G, uint8_t B,
                                     timeus_t interval_ms, timeus_t dT_us,
                                     float duty)
** 函数说明： LED呼吸灯
**---------------------------------------------------------------------
** 输入参数： 颜色（RGB），闪灯间隔，更新间隔、占空比
** 返回参数： 无
***********************************************************************/
void LEDControlTask::breath(uint8_t R, uint8_t G, uint8_t B, timeus_t interval_ms, timeus_t dT_us)
{
  static timeus_t timer_us = 0;
  volatile static timeus_t pwm_cycle_ms = 25;
  volatile static timeus_t pwm_pulse_ms = 0;
  if(timer_us >= interval_ms * 1000)
  {
    timer_us = 0;
    pwm_pulse_ms = 0;
  }
  else if(timer_us < interval_ms * 1000 * 0.1)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0);
  }
  else if(timer_us >= interval_ms * 1000 * 0.1 && timer_us < interval_ms * 1000 * 0.2)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0.2);
  }
  else if(timer_us >= interval_ms * 1000 * 0.2 && timer_us < interval_ms * 1000 * 0.3)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0.4);
  }
  else if(timer_us >= interval_ms * 1000 * 0.3 && timer_us < interval_ms * 1000 * 0.4)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0.6);
  }
  else if(timer_us >= interval_ms * 1000 * 0.4 && timer_us < interval_ms * 1000 * 0.5)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0.8);
  }
  else if(timer_us >= interval_ms * 1000 * 0.5 && timer_us < interval_ms * 1000 * 0.6)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 1);
  }
  else if(timer_us >= interval_ms * 1000 * 0.6 && timer_us < interval_ms * 1000 * 0.7)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0.9);
  }
  else if(timer_us >= interval_ms * 1000 * 0.7 && timer_us < interval_ms * 1000 * 0.8)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0.6);
  }
  else if(timer_us >= interval_ms * 1000 * 0.8 && timer_us < interval_ms * 1000 * 0.9)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0.4);
  }
  else if(timer_us >= interval_ms * 1000 * 0.9 && timer_us < interval_ms * 1000 * 1.0)
  {
    blink(R, G, B, pwm_cycle_ms, dT_us, 0);
  }
  timer_us += dT_us;
}
