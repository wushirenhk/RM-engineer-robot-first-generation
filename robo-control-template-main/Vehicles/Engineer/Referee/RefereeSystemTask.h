#ifdef ENGINEER
#ifndef __REFEREESYSTEMTASK_H__
#define __REFEREESYSTEMTASK_H__

#include "RefereeSystem_Data.h"
#include "Scheduler_Common.h"
#include <cstring>
#include "Scheduler.h"
#include "UARTDriver.h"

class Robot;

struct Transmit_DMAFunc : AsyncFuncBase
{
	uint8_t send_cnt = 0;
	UART_HandleTypeDef *huart;
	uint8_t *pData;
	uint32_t Size;
	
	
	Transmit_DMAFunc& setData(UART_HandleTypeDef *huart_, uint8_t *pData_, uint32_t Size_)
	{
		
		send_cnt = 0;
		delay_ms = Size *8.0f / 5000 * 1000;
		this->huart = huart_;
		this->pData = pData_;
		this->Size = Size_;
		return *this;
	}
	
	virtual void operator()()//方法，可以将对象名当作函数名使用
	{
	  HAL_UART_Transmit_DMA(huart, pData , Size);
		//USART1_DMA_Debug_Printf("h %d\n", Size);
//		
//		for(int i = 0; i < Size; i ++)
//		{
//			USART1_DMA_Debug_Printf("c %d 0x%x\n", i, *(pData + i));
//			HAL_Delay(10);
//		}
		if(send_cnt--)
		{
			delay_ms = Size *8.0f / 5000 * 1000;
			reset();
		}
		else run = true;
	}		
};

class RefereeSystemTask : public Task_Base
{
public:
	friend class RemoteControlTask;
  RefereeSystemTask(Robot &robot0, timeus_t interval_tick_us0 = 1e6f / 100.0f) : Task_Base(robot0)
  {
    this->interval_tick_us = interval_tick_us0;
  }
  virtual ~RefereeSystemTask(void) {}
  void init(void);
  void update(timeus_t dT_us);
  void uninit(void);
    
  uint8_t referee_buf[REFEREE_BUF_NUM];
  uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_NUM][REFEREE_FIFO_BUF_LEN];

  void pushToBuffer(uint16_t start_pos, uint16_t end_pos);
  void parseData(void);
  RobotRefereeStatus_t getRobotRefereeStatus_t(void)
	{
		return robot_referee_status;
	}
	void set_num_0()
	{
		num = 0;
	}
	void set_t_0()
	{
		t = 0;
	}
	void create_ui_interactive_package(uint8_t *tx_buf, uint8_t tx_buf_len, uint16_t data_cmd_id);
	void set_clear_graphics_data(uint8_t *tx_buf, uint8_t operate, uint8_t layer);
	void set_line_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);
	void set_rectangle_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_x, uint16_t start_y, uint16_t opposite_x, uint16_t opposite_y);
	void set_circle_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t center_x, uint16_t center_y, uint16_t radius);
	void set_ellipse_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y);
	void set_arc_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_angle, uint16_t end_angle, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y);
	void ClientUI_ClearLayer(uint8_t layer);
	void ClientUI_DrawLine(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);
	void ClientUI_DrawRectangle(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_x, uint16_t start_y, uint16_t opposite_x, uint16_t opposite_y);
	void ClientUI_DrawCircle(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t center_x, uint16_t center_y, uint16_t radius);
	void ClientUI_DrawEllipse(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y);
	void ClientUI_DrawArc(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_angle, uint16_t end_angle, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y);
	void Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData,uint32_t Size);
	void ClientUI_DrawString(uint8_t layer, char *name, uint8_t operate, uint8_t size, uint8_t color, uint16_t start_x, uint16_t start_y, const char *str, ...);
	void set_DrawString_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint8_t size, uint8_t color, uint16_t start_x, uint16_t start_y, const char *str, ...);
	void Refresh_ClientUI();
	void RefreshDynamicClientUI();
	void float_to_str(char *str,float num);
protected:
  uint8_t fifo_count;
  uint8_t fifo_head_pos;
  uint8_t fifo_tail_pos;
  uint8_t referee_seq_num;
  uint8_t student_interactive_seq;
  uint8_t num;
	uint8_t t;
  RobotRefereeStatus_t robot_referee_status;
	Transmit_DMAFunc transmit_dma_func;
  uint16_t hz = 1;
	uint16_t x1 = 250;
	uint16_t x2 = 500;
	uint16_t y1 = 50;
	uint16_t y2 = 200;
};

#endif
#endif