#ifndef CANDEVICE_H
#define CANDEVICE_H

#include "main.h"
#include "Error.h"

#define MAX_CAN_RX_ID_NUM 15
#define MAX_CAN_TX_ID_NUM 15

//enum CAN_LINE
//{
//  CAN_LINE1 = 1,
//  CAN_LINE2 = 2
//};

///* CAN send and receive ID */
//typedef enum
//{

//  CAN_AMMO_CTRL_ID = 0x200,
//  CAN_BREAK_CTRL_ID = 0x1FF,
//  CAN_GIMBAL_CTRL_ID = 0x1FF,
//  CAN_MAIN_MOTOR_CTRL_ID = 0x200,
//  CAN_BOARD_C_MID_CTRL_ID = 0x401,
//  CAN_BOARD_C_DWN_CTRL_ID = 0x402,
//  CAN_MINIPC_DWN_EULER_SYNC_ID = 0x403,
//  CAN_MINIPC_DWN_CTRL_ID = 0x404,

//  /*CAN2_UP*/
//  CAN_TRIGGER_UP_ID = 0x202,
//  CAN_3508_MAIN_ID = 0x201,
//  CAN_AMMO_BOOSTER_UP_0_ID = 0x203,
//  CAN_AMMO_BOOSTER_UP_1_ID = 0x204,
//  CAN_BREAK_0_ID = 0x205,
//  CAN_BREAK_1_ID = 0x206,

//  /*CAN2_DWN*/
//  CAN_AMMO_BOOSTER_DWN_0_ID = 0x201,
//  CAN_AMMO_BOOSTER_DWN_1_ID = 0x202,
//  CAN_MINIPC_DWN_ID = 0x500,

//  /*CAN1_UP_DWN*/
//  CAN_TRIGGER_DWN_ID = 0x202,
//  CAN_6020_Z_DWN_ID = 0x205,
//  CAN_6020_Y_DWN_ID = 0x206,
//  CAN_6020_Z_UP_ID = 0x207,
//  CAN_6020_Y_UP_ID = 0x208,

//} can_msg_id_e;

//CANRX包：ID,DLC,DATA
typedef struct _CAN_Rx_Data_Pack_t
{
  uint16_t std_id;
  uint8_t dlc;
  uint8_t data[8];
  uint16_t data_0 = 0x0FFF;
  uint8_t data_1 = 0x0F;
} CAN_Rx_Data_Pack_t;

//CANTX包：ID,DLC,DATA
typedef struct _CAN_Tx_Data_Pack_t
{
  uint16_t std_id;
  uint8_t dlc;
  uint8_t* data[8];
} CAN_Tx_Data_Link_t;


class CanDevice
{
public:
  CanDevice();
  void init(CAN_HandleTypeDef *_hcanx);

//  void setRxPointerNum(uint16_t num);
  void send(CAN_TxHeaderTypeDef *header, uint8_t *data);
  void send(uint16_t std_id, uint8_t *data);
  bool addRxLink(CAN_Rx_Data_Pack_t* addr);
  bool addTxLink(
    uint16_t can_tx_id,
    uint8_t can_tx_dlc,
    uint8_t start_pos,
    uint8_t* data_p,
    uint8_t size
  );
  // void CanDevice::initFilter(void);

  uint8_t can_rx_link_count;
  uint8_t can_tx_link_count;

	//CAN接收数据包结构体指针数组
  CAN_Rx_Data_Pack_t* can_rx_data_p[MAX_CAN_RX_ID_NUM];
	//CAN发送数据包结构体数组
  CAN_Tx_Data_Link_t can_tx_data[MAX_CAN_TX_ID_NUM];

	
  const CAN_TxHeaderTypeDef standard_tx_header =
  {
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 0x08
  };

protected:
  CAN_HandleTypeDef *hcanx;
};


#endif
