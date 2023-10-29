/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   Helper.cpp
** 文件说明：   板载LED、5v接口、蜂鸣器、舵机控制
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	   2022-07-08
**	 1.1							   补充注释						     赵钟源     	   2022-12-10
***************************************************************************/

#include "Helper.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim4;

/***********************************************************************
** 函 数 名： Helper::setBuzzerOff
** 函数说明： 关闭蜂鸣器
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::setBuzzerOff(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}

/***********************************************************************
** 函 数 名： Helper::setBuzzerFrequence
** 函数说明： 设置蜂鸣器频率
**---------------------------------------------------------------------
** 输入参数： freq频率
** 返回参数： 无
***********************************************************************/
void Helper::setBuzzerFrequence(uint16_t freq)
{
  uint16_t period = 1000000 / freq - 1;

  __HAL_TIM_SET_AUTORELOAD(&htim4, period);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period / 20);
}

/***********************************************************************
** 函 数 名： Helper::setBuzzerState
** 函数说明： 设置蜂鸣器状态
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
int8_t Helper::setBuzzerState(uint8_t state)
{
	//PLAYING_INIT_MUSIC不能被打断，除非ERROR
  if((state != PLAYING_ERROR_SOUND) && (buzzer_state == PLAYING_INIT_MUSIC))
  {
    return -1;
  }
  
  buzzer_state = state;
  if(state == PLAYING_STOP)
  {
    setBuzzerOff();
  }

  bzply_n = 0;
  bzply_count = 1;

  return 0;
}

/***********************************************************************
** 函 数 名： Helper::playingSound
** 函数说明： 
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::playingSound(uint8_t *sound, uint16_t len)
{
  uint8_t delay, freq;

  delay = ((sound[bzply_n] & 0xF0) >> 4) * 2;
  freq = (sound[bzply_n] & 0x0F);

  if(bzply_count < delay)
  {
    bzply_count++;
    if(freq == 0x00)
    {
      setBuzzerOff();
    }
    else if(freq == 0x0B)
    {
      setBuzzerFrequence(640);
    }
    else if(freq == 0x0D)
    {
      setBuzzerFrequence(256);
    }
  }
  else
  {
    bzply_count = 1;
    bzply_n++;
    if(bzply_n >= len)
    {
      bzply_n = 0;
    }
  }
}

/***********************************************************************
** 函 数 名： Helper::playingSong
** 函数说明： 播放歌曲
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
bool Helper::playingSong(uint16_t *song, uint16_t len)
{
  uint8_t off_delay, on_delay, level, step;

  off_delay = ((song[bzply_n] & 0xF000) >> 12);
  on_delay = ((song[bzply_n] & 0x0F00) >> 8) * 6;
  level = ((song[bzply_n] & 0x00F0) >> 4);
  step = (song[bzply_n] & 0x000F) - 1;

  if(bzply_count < on_delay)
  {
    bzply_count++;
    setBuzzerFrequence(music_steps[level][step]);
  }
  else if(bzply_count < (on_delay + off_delay))
  {
    bzply_count++;
    setBuzzerOff();
  }
  else
  {
    bzply_count = 1;
    bzply_n++;
    if(bzply_n >= len)
    {
      bzply_n = 0;
      buzzer_state = PLAYING_STOP;
      setBuzzerOff();
      return true;
    }
  }
  return false;
}

/***********************************************************************
** 函 数 名： Helper::setLED
** 函数说明： 设置LED开关
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::setLED(uint8_t R, uint8_t G, uint8_t B)
{
  if (R == 1)
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  }
  else if(R == 0)
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  }
  if (G == 1)
  {
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  }
  else if(G == 0)
  {
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  }
  if (B == 1)
  {
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
  }
  else if(B == 0)
  {
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  }
}
/***********************************************************************
** 函 数 名： Helper::toggleLED
** 函数说明： 反转LED
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::toggleLED(uint8_t R, uint8_t G, uint8_t B)
{
  if (R == 1)
  {
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
  }
  if (G == 1)
  {
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  }
  if (B == 1)
  {
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
  }
}
/***********************************************************************
** 函 数 名： Helper::toggleAllLED
** 函数说明： 反转所有LED
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::toggleAllLED(void)
{
  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}
/***********************************************************************
** 函 数 名： Helper::setLaser
** 函数说明： 开启激光器
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::setLaser(void)
{
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}
/***********************************************************************
** 函 数 名： Helper::resetLaser
** 函数说明： 关闭激光器
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::resetLaser(void)
{
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}
/***********************************************************************
** 函 数 名： Helper::toggleLaser
** 函数说明： 开关激光器
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::toggleLaser(void)
{
  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
}
/***********************************************************************
** 函 数 名： Helper::setServo
** 函数说明： 控制舵机
**---------------------------------------------------------------------
** 输入参数： number舵机id,degree旋转角度
** 返回参数： 无
***********************************************************************/
void Helper::setServo(uint8_t number, float degree)
{
  uint16_t serp;

  switch(number)//
  {
  case(1):
	 serp = 500 + ((2000 * degree) / 135); //角度换算成占空比
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, serp);
    break;
  case(2):
	  serp = 500 + ((2000 * degree) / 270); //角度换算成占空比
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, serp);
    break;
  
    case(3):
	  serp = 500 + ((2000 * degree) / 270); //角度换算成占空比
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, serp);
    break;
  }
}
/***********************************************************************
** 函 数 名： Helper::heat
** 函数说明： PWM加热
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Helper::heat(uint16_t out)
{
  __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 2000);
}
