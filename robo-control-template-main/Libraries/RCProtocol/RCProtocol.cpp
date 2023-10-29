/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   RCProtocol.cpp
** 文件说明：   遥控器类
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	     2022-07-18
**	 1.1							   补充注释						     赵钟源     	     2022-12-10
***************************************************************************/

#include "RCProtocol.h"
#include "RCProtocol_Backend.h"
#include "RCProtocol_DBUS.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

//uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/***********************************************************************
** 函 数 名： RCProtocol::init
** 函数说明： huart3与DMA初始化，实例化后台，并添加
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RCProtocol::init(void)
{
  addBackend(new RCProtocol_DBUS(*this));//将DBUS添加为新后台
  backend->init();//初始化DBUS(空)
  //enable the DMA transfer for the receiver request
  //使能DMA串口接收
  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

  //enalbe idle interrupt
  //使能空闲中断
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

  //disable DMA
  //失效DMA
  __HAL_DMA_DISABLE(&hdma_usart3_rx);
  while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
  }

  hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
  //memory buffer 1

  //memory buffer 2

  //内存缓冲区1
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(sbus_rx_buf[0]);
  //内存缓冲区2
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(sbus_rx_buf[1]);
  //data length
  //数据长度
  hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
  //enable double memory buffer
  //使能双缓冲区
  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

  //enable DMA
  //使能DMA
  __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

/***********************************************************************
** 函 数 名： RCProtocol::processByte
** 函数说明： 调用后台的processByte函数
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RCProtocol::processByte(volatile const uint8_t *buf)
{
  if(backend != NULL)
  {
    backend->processByte(buf);
  }
}

/***********************************************************************
** 函 数 名： RCProtocol::addBackend
** 函数说明： 在前台添加新后台
**---------------------------------------------------------------------
** 输入参数： 后台类指针backend0
** 返回参数： 无
***********************************************************************/
void RCProtocol::addBackend(RCProtocol_Backend *backend0)
{
  backend = backend0;
}

/***********************************************************************
** 函 数 名： RCProtocol::uninit
** 函数说明： 在前台调用后台的去初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RCProtocol::uninit(void)
{
  backend->uninit();
}
