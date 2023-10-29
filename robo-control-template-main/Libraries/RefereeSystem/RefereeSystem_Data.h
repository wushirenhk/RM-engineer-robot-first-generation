#ifndef __REFERENCESYSTEM_DATA_H__
#define __REFERENCESYSTEM_DATA_H__

#include <stdio.h>
#include <stdarg.h>
#include "stdint.h"
#include "main.h"
#include "Scheduler_Common.h"

#define REFEREE_BUF_NUM 1024
#define REFEREE_BUF_LEN 1024
#define REFEREE_FIFO_BUF_NUM				8
#define REFEREE_FIFO_BUF_LEN 				1024

/* 数据帧帧头结构体 */
typedef __PACKED_STRUCT
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

/* ----------------------------------比赛数据帧的数据段框架--------------------------------------- */
/**
  @brief  比赛状态数据：0x0001，1Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t game_type: 4;
  uint8_t game_progress: 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
  timeus_t timestamp;
} ext_game_status_t;

/**
   @brief 比赛结果数据：0x0002，在比赛结束后发送
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t winner;
  timeus_t timestamp;
} ext_game_result_t;

/**
   @brief 机器人血量数据：0x0003，1Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
  timeus_t timestamp;
} ext_game_robot_HP_t;

/**
   @brief 场地事件数据：0x0101，事件改变发送
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint32_t event_type;
  timeus_t timestamp;
} ext_event_data_t;

/**
   @brief 补给站动作标识：0x0102	动作改变后发送
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
  timeus_t timestamp;
} ext_supply_projectile_action_t;

/**
   @brief 裁判警告信息：0x0104	警告发生后发送
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t level;
  uint8_t foul_robot_id;
  timeus_t timestamp;
} ext_referee_warning_t;

/**
   @brief 当前比赛机器人状态：0x0201，10Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_HP;
  uint16_t max_HP;
  uint16_t shooter_id1_17mm_cooling_rate;
  uint16_t shooter_id1_17mm_cooling_limit;
  uint16_t shooter_id1_17mm_speed_limit;
  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
  uint16_t chassis_power_limit;
  uint8_t mains_power_gimbal_output: 1;
  uint8_t mains_power_chassis_output: 1;
  uint8_t mains_power_shooter_output: 1;
  timeus_t timestamp;
} ext_game_robot_status_t;

/**
   @brief 实时功率热量数据：0x0202，50Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t chassis_power_buffer;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
  timeus_t timestamp;
} ext_power_heat_data_t;

/**
   @brief 机器人位置：0x0203,10Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  float x;
  float y;
  float z;
  float yaw;
  timeus_t timestamp;
} ext_game_robot_pos_t;

/**
   @brief 机器人增益：0x0204，状态改变后发送
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t power_rune_buff;
  timeus_t timestamp;
} ext_buff_t;

/**
   @brief 伤害状态：0x0206，受到伤害后发送
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t armor_id: 4;
  uint8_t hurt_type: 4;
  timeus_t timestamp;
} ext_robot_hurt_t;

/**
   @brief 实时射击信息：0x0207，射击后发送
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt, recv_cnt_last;
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
  float bullet_speed_last;
  timeus_t timestamp;
} ext_shoot_data_t;

/**
   @brief 子弹剩余发射数：0x0208，10Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
  timeus_t timestamp;
} ext_bullet_remaining_t;

/**
   @brief 机器人RFID状态：0x0209，1Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint32_t rfid_status;
  timeus_t timestamp;
} ext_rfid_status_t;

/**
   @brief 机器人裁判系统获取总状态数据
*/
typedef __PACKED_STRUCT
{
  ext_game_status_t game_status; //0x0001
  ext_game_result_t game_result; //0x0002
  ext_game_robot_HP_t game_robot_HP; //0x0003

  ext_event_data_t event_data; //0x0101
  ext_supply_projectile_action_t supply_projectile_action; //0x0102
  ext_referee_warning_t referee_warning; //0x0104

  ext_game_robot_status_t game_robot_status; //0x0201
  ext_power_heat_data_t power_heat_data; //0x0202
  ext_game_robot_pos_t game_robot_pos; //0x0203
  ext_buff_t buff; //0x0204

  ext_robot_hurt_t robot_hurt; //0x0206
  ext_shoot_data_t shoot_data; //0x0207
  ext_bullet_remaining_t bullet_remaining; //0x0208
  ext_rfid_status_t rfid_status; //0x0209


  uint32_t cmd_error_count[6]; //0-seq_num error; 1-frame_header & length error; 2-head_crc8 error; 3-package length error; 4-package_crc16 error; 5-cmd error
} RobotRefereeStatus_t;

/* ----------------------------------裁判系统客户端交互部分--------------------------------------- */
/**
	 @brief 机器人间交互数据:0x0301,10HZ
*/
typedef __PACKED_STRUCT
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/**
	 @brief 机器人间交互数据:0x0301,10HZ
	 @brief 客户端删除图形，数据内容ID:0x0101
*/ 
typedef __PACKED_STRUCT
{
	uint8_t operate_type;
	uint8_t layer;
} ext_client_custom_graphic_delete_t;

/**
	 @brief 图形数据，15字节
*/
typedef __PACKED_STRUCT
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	__packed union{
		__packed struct{
			uint32_t radius:10;
			uint32_t end_x:11;
			uint32_t end_y:11;
		} value;
		float float_value;
		int32_t int32_value;
	} graphic_config_3;
} graphic_data_struct_t;

/**
	 @brief 机器人间交互数据:0x0301,10HZ
	 @brief 客户端绘制字符，数据内容ID：0x0110
*/
typedef __PACKED_STRUCT
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;

/**
	 @brief 客户端UI所需数据
*/
typedef __PACKED_STRUCT
{
	
}ClientUIStatus_t;


/*---------------------DECLARES----------------------*/
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
