/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   Scheduler_Common.cpp
** 文件说明：   任务调度器共用
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	   2022-07-08
***************************************************************************/

#include "Scheduler_Common.h"
#include "UARTDriver.h"

#define TICK_PER_SECOND	1000
#define TICK_US	(1000000/TICK_PER_SECOND)

inline static timeus_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
static timeus_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  // GXT_SYSTICK_IsActiveCounterFlag();
  timeus_t m;
	timeus_t value;
//  const timeus_t tms = SysTick->LOAD + 1;
//  __IO timeus_t u = tms - SysTick->VAL;
  do
  {
    m = HAL_GetTick();
		value = m * TICK_US + ( SysTick->LOAD - SysTick->VAL ) * TICK_US / SysTick->LOAD;
  }
	while (m != HAL_GetTick());
	// if((u * 1000) / tms > 500) m -= 1;
  return value;
}

static timeus_t getCurrentMicrosTim5(void)
{
  return TIM5->CNT;
}

void DWT_Init(void)
{
	CoreDebug->DEMCR   |= CoreDebug_DEMCR_TRCENA_Msk; \
    DWT->CYCCNT         = 0; \
    DWT->CTRL          |= DWT_CTRL_CYCCNTENA_Msk;
}

//获取系统时间，单位us
timeus_t micros(void)
{
	return getCurrentMicrosTim5();
	//return (DWT->CYCCNT / (SystemCoreClock / 1000000U));
//	static timeus_t last = 0, now = 0;
//	last = now;
//	now = getCurrentMicros();
//	
//	if(now < last)
//	{
//		Debug_Printf("!!!!!!\r\n");
//	}
//	
//  return now;
}

// 判断是否超时
bool isOutOfClock(timeus_t timestamp, timeus_t interval_max)
{
  return micros() - timestamp > interval_max;
}
