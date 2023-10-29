#ifndef ERROR_H
#define ERROR_H
#include "stdint.h"

typedef struct _CAN_Data_Conflict_Error_Code_t
{
  uint8_t error:1;
  uint8_t can_device_id;
  uint16_t error_can_addr;
  uint8_t error_data_pos;
} CAN_Data_Conflict_Error_Code_t;


typedef struct _Error_Code_t
{
  uint8_t can_tx_data_out_of_range:1;
  uint8_t can_tx_id_num_out_of_range:1;
  uint8_t can_rx_id_num_out_of_range:1;
  uint8_t can_tx_dlc_mismatch:1;
  uint8_t inertial_sensor_backend_id_out_of_range:1;
  uint8_t inertial_sensor_find_id_out_of_range:1;
  uint8_t pid_controller_adder_not_initialized:1;
  CAN_Data_Conflict_Error_Code_t can_tx_data_conflict;
} Error_Code_t;

class Error
{
public:
  Error() {}
  void Error_Handler();
  Error_Code_t error_code;

};

extern Error error;

#endif