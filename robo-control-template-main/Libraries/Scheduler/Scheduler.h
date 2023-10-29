#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "stdint.h"
#include "Task.h"
#include "Scheduler_Common.h"

class Robot;

struct AsyncFuncBase
{
	timeus_t register_ms;
	timeus_t delay_ms;
	bool run;
	virtual void operator()()
	{
		run = true;
	};
	void reset()
	{
		run = false;
		register_ms = micros() / 1000;
	}
};

class Scheduler
{
public:
  Scheduler();
  void init(void);
  void run(void);
  uint16_t getTaskNum(void);
  bool registerTask(Task_Base *task_p);
  bool unregisterTask(Task_Base **task_p_p);
  timeus_t getSysTimeUs(void);
	void startAsync(AsyncFuncBase &func, timeus_t delay_ms)
	{
		func.run = false;
		func.delay_ms = delay_ms;
		func.register_ms = getSysTimeUs() / 1000;
		uint16_t i;
		for(i = 0; i < async_num; i ++)
		{
			if(asyncs[i] == &func) return;
		}
		if(i == async_num) asyncs[async_num++] = &func;
	}


private:
  Task_Base *tasks[MAX_TASK_NUM];
  uint16_t task_num;
	AsyncFuncBase *asyncs[MAX_ASYNC_NUM];
	uint16_t async_num;
};

#endif

