/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   Scheduler.cpp
** 文件说明：   任务调度器
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	   2022-07-08
***************************************************************************/


#include "Scheduler.h"

Scheduler::Scheduler(void)
{

}

/***********************************************************************
** 函 数 名： Scheduler::getTaskNum()
** 函数说明： 获取当前任务数量
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 任务数量
***********************************************************************/
uint16_t Scheduler::getTaskNum(void)
{
  return task_num;
}

/***********************************************************************
** 函 数 名： Scheduler::registerTask()
** 函数说明： 在调度器中注册该任务
**---------------------------------------------------------------------
** 输入参数： 任务指针
** 返回参数： 注册是否成功
***********************************************************************/
bool Scheduler::registerTask(Task_Base *task_p)
{
  uint8_t i;
  if(task_p == NULL) return false;

  /* 防止重复添加任务 */
  for(i = 0; i < task_num; i++)
  {
    if(task_p == tasks[i])
    {
      return false;
    }
  }
  /* 防止重复添加任务 */

  if(task_num >= MAX_TASK_NUM)
  {
    return false;
  }

  tasks[task_num++] = task_p;
  return true;
}

//  TODO 注销任务 unRegisterTask(Task_Base *task_p)
/***********************************************************************
** 函 数 名： Scheduler::unregisterTask()
** 函数说明： 在调度器中注销该任务
**---------------------------------------------------------------------
** 输入参数： 任务指针
** 返回参数： 注销是否成功
***********************************************************************/
bool Scheduler::unregisterTask(Task_Base **task_p_p)
{
  if(*task_p_p == NULL)
  {
    return false;
  }

  (*task_p_p)->uninit();

  uint16_t i = 0;
  bool found_flag = false;

  for(i = 0; i < task_num; i += 1)//查询任务后注销任务
  {
    if(tasks[i] == *task_p_p)
    {
      delete *task_p_p;
      tasks[i] = NULL;
      *task_p_p = NULL;
      found_flag = true;
      break;
    }
  }

  if(found_flag)//注销任务后，后续任务指针前移
  {
    for(; i < task_num - 1; i ++)
    {
      tasks[i] = tasks[i + 1];
    }
    tasks[i] = NULL;
  }

  task_num --;

  return true;
}


/***********************************************************************
** 函 数 名： Scheduler::getSysTimeUs()
** 函数说明： 获取系统时间，单位us
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
timeus_t Scheduler::getSysTimeUs(void)
{
  return micros();
}

/***********************************************************************
** 函 数 名： Scheduler::init()
** 函数说明： 调度器初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Scheduler::init(void)
{

}

/***********************************************************************
** 函 数 名： Scheduler::run()
** 函数说明： 调度器运行函数，此函数运行在主循环中
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Scheduler::run(void)
{
  for(uint16_t i = 0; i < async_num; i ++)
	{
    timeus_t time_now_ms = micros() / 1000;
		if(asyncs[i]->run != true && asyncs[i]->register_ms > 0 && time_now_ms - asyncs[i]->register_ms > asyncs[i]->delay_ms && time_now_ms - asyncs[i]->delay_ms < 0xffffffff)
		{
			(*asyncs[i])();
			//asyncs[i]->run = true;
		}
	}
  for(uint16_t i = 0; i < task_num; i ++)
  {
    timeus_t time_now_us = micros();
    timeus_t delta_time_us = (timeus_t)(time_now_us - tasks[i]->last_tick_us);

    // 判断当前时间距离上次执行的时间是否达到设定的间隔运行时间
    if((delta_time_us >= tasks[i]->interval_tick_us) && (tasks[i]->interval_tick_us))
    {
      // 防止delta_time_us发疯带来不必要的损失，跳过本次任务
      if(delta_time_us >= 0xfffffff)
      {
        tasks[i]->last_tick_us = time_now_us;
        continue;
      }
#ifdef DEBUG_MODE
      tasks[i]->update_freq = 1e6f / delta_time_us;
#endif
      // 更新任务的执行时间
      tasks[i]->last_tick_us = micros();

      // 判断任务是否已初始化
      if(tasks[i]->inited)
      {
        tasks[i]->update(delta_time_us);//运行每个任务中的updata函数
      }
      else
      {
        tasks[i]->inited = true;
        tasks[i]->init();
      }
      //判断任务是否需要注销
      if(tasks[i]->self_kill == true)
      {
        tasks[i]->self_kill = false;
        unregisterTask(&tasks[i]);
      }
      tasks[i]->end_tick_us = micros();
      tasks[i]->real_interval_tick_us = tasks[i]->end_tick_us - time_now_us;
    }

  }
}
