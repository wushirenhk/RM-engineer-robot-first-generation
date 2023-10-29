#ifndef __SCHEDULER_COMMON_H__
#define __SCHEDULER_COMMON_H__

#include "main.h"
#include "stdint.h"

#define timeus_t uint32_t
#define TIMEUS_MAX UINT32_MAX
#define MAX_TASK_NUM 100
#define MAX_ASYNC_NUM 500

//获取系统时间，单位us
timeus_t micros(void);
void DWT_Init(void);
bool isOutOfClock(timeus_t timestamp, timeus_t interval_max = 50000);

#endif
