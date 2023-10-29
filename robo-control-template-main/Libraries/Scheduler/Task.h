#ifndef __TASK_H__
#define __TASK_H__

#include "stdint.h"
#include "Scheduler_Common.h"

class Robot;
// 任务基类，所有的任务需继承自此类
class Task_Base//抽象类
{
  // 设置调度器类为友元类，使调度器类可以访问自己的非公开成员
  friend class Scheduler;

public:
  Task_Base(Robot &robot0) : robot(robot0)//构造函数，将引用参数&robot0赋值给属性robot
  {
  }

  virtual ~Task_Base(void) {};

  // 纯虚函数，子类必须实现
  virtual void update(timeus_t dT_us) = 0; // 间隔interval_tick_us时间进行轮询，输入参数dT_us为实际运行间隔时间
  virtual void init(void) = 0; // 任务初始化时执行此函数
  virtual void uninit(void) = 0; // 任务销毁时执行此函数
  
		//是否已经初始化
		bool isInited(void)
  {
    return inited;
  }
	
	//是否自杀
  bool isKilledBySelf(void)
  {
    return self_kill;
  }
	//自杀
  void selfKill(void)
  {
    self_kill = true;
  }
	//设置时间间隔
  void setInterval(timeus_t interval)
  {
    interval_tick_us = interval;
  }
	//返回时间间隔
  timeus_t getInterval(void)
  {
    return interval_tick_us;
  }
//  void setInitState(bool state)
//  {
//    inited = state;
//  }
protected:
  bool inited;
#ifdef DEBUG_MODE
  float update_freq;
#endif
  timeus_t interval_tick_us; // 间隔运行时间，若设置成0，则不会在Scheduler中运行该任务
  timeus_t real_interval_tick_us; // 真实的运行间隔
  timeus_t last_tick_us;	// 上次运行的时间戳
  timeus_t end_tick_us; // 此次运行结束时间戳
  Robot &robot; // 所属的机器人对象
  bool self_kill;
};

#endif
