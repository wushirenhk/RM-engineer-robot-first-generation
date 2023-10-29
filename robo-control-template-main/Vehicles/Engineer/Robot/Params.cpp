	#include "Params.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/***********************************************************************
** �� �� ���� Params::initMotorsParams()
** ����˵���� ��ʼ���������������PID����������ʱ�����ڲ���
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Params::initMotorsParams()
{
	
  /************** RM ��� CAN���ߵ�ַ **************/

  motor_params.chassis_motor_1 = {.can_tx_id = 0x200, .can_rx_id = 0x201, .can_tx_data_start_pos = 0, .canx = 2};
  motor_params.chassis_motor_2 = {.can_tx_id = 0x200, .can_rx_id = 0x202, .can_tx_data_start_pos = 2, .canx = 2};
  motor_params.chassis_motor_3 = {.can_tx_id = 0x200, .can_rx_id = 0x203, .can_tx_data_start_pos = 4, .canx = 2};
  motor_params.chassis_motor_4 = {.can_tx_id = 0x200, .can_rx_id = 0x204, .can_tx_data_start_pos = 6, .canx = 2};
 
  motor_params.gimbal_6020 =     {.can_tx_id = 0x1FF, .can_rx_id = 0x205, .can_tx_data_start_pos = 0, .canx = 2};
  motor_params.arm_3508 =        {.can_tx_id = 0x1FF, .can_rx_id = 0x206, .can_tx_data_start_pos = 2, .canx = 2};
  motor_params.overturn1_3508 =  {.can_tx_id = 0x1FF, .can_rx_id = 0x207, .can_tx_data_start_pos = 4, .canx = 2};
  motor_params.overturn2_3508 =  {.can_tx_id = 0x1FF, .can_rx_id = 0x208, .can_tx_data_start_pos = 6, .canx = 2};
  
  /************** RM ��� CAN���ߵ�ַ **************/
  
  
  /************** MC ��� CAN���ߵ�ַ **************/  
 
  
  /************** MC ��� CAN���ߵ�ַ **************/
  
  
  /************** DM ��� CAN���ߵ�ַ **************/  
	
  motor_params.arm_dm_1 = {.can_tx_id = 0x101, .can_rx_id = 0x01, .can_tx_data_start_pos = 0, .canx = 1};
  
  motor_params.arm_dm_2 = {.can_tx_id = 0x102, .can_rx_id = 0x02, .can_tx_data_start_pos = 0, .canx = 1};  
  
  /************** DM ��� CAN���ߵ�ַ **************/
  
  /************** M15 ��� CAN���ߵ�ַ **************/  
	
  motor_params.arm_M15_1 = {.can_tx_id = 0x32, .can_rx_id = 0x97, .can_tx_data_start_pos = 0, .canx = 1};
  
  /************** DM ��� CAN���ߵ�ַ **************/
  
  /**************  �������  **************/
  
  	  /************** MG ��� CAN���ߵ�ַ **************/  
  
  motor_params.arm_mg_1 = {.can_tx_id = 0x145, .can_rx_id = 0x145, .can_tx_data_start_pos = 0, .canx = 1};
  
//  
  /************** MG ��� CAN���ߵ�ַ **************/
  


  // �ĸ����̵���������
  motor_params.chassis_motor_1.reduction_ratio = 19.0f;
  motor_params.chassis_motor_1.output_radius = .075f;
  motor_params.chassis_motor_1.direction = MOTOR_CW;
  motor_params.chassis_motor_1.max_value_ecd = 8192;
  motor_params.chassis_motor_1.offset_ecd = 0;

  motor_params.chassis_motor_2.reduction_ratio = 19.0f;
  motor_params.chassis_motor_2.output_radius = .075f;
  motor_params.chassis_motor_2.direction = MOTOR_CW;
  motor_params.chassis_motor_2.max_value_ecd = 8192;
  motor_params.chassis_motor_2.offset_ecd = 0;

  motor_params.chassis_motor_3.reduction_ratio = 19.0f;
  motor_params.chassis_motor_3.output_radius = .075f;
  motor_params.chassis_motor_3.direction = MOTOR_CW;
  motor_params.chassis_motor_3.max_value_ecd = 8192;
  motor_params.chassis_motor_3.offset_ecd = 0;

  motor_params.chassis_motor_4.reduction_ratio = 19.0f;
  motor_params.chassis_motor_4.output_radius = .075f;
  motor_params.chassis_motor_4.direction = MOTOR_CW;
  motor_params.chassis_motor_4.max_value_ecd = 8192;
  motor_params.chassis_motor_4.offset_ecd = 0;
    
  //��е��6020�������
  motor_params.gimbal_6020.reduction_ratio = 1.0f;
  motor_params.gimbal_6020.output_radius = 1.0f;
  motor_params.gimbal_6020.direction = MOTOR_CW;
  motor_params.gimbal_6020.max_value_ecd = 8192;
  motor_params.gimbal_6020.offset_ecd = 0;
  
  //��е��3508�������
  motor_params.arm_3508.reduction_ratio = 19.0f;
  motor_params.arm_3508.output_radius = .075f;
  motor_params.arm_3508.direction = MOTOR_CW;
  motor_params.arm_3508.max_value_ecd = 8192;
  motor_params.arm_3508.offset_ecd = 0;
	 
  //����3508�������
  motor_params.overturn1_3508.reduction_ratio = 19.0f;
  motor_params.overturn1_3508.output_radius = .075f;
  motor_params.overturn1_3508.direction = MOTOR_CW;
  motor_params.overturn1_3508.max_value_ecd = 8192;
  motor_params.overturn1_3508.offset_ecd = 0;

  motor_params.overturn2_3508.reduction_ratio = 19.0f;
  motor_params.overturn2_3508.output_radius = .075f;
  motor_params.overturn2_3508.direction = MOTOR_CW;
  motor_params.overturn2_3508.max_value_ecd = 8192;
  motor_params.overturn2_3508.offset_ecd = 0;
  
  //DM�������
  motor_params.arm_dm_1.reduction_ratio = 10;
  motor_params.arm_dm_1.output_radius = 1;  
  motor_params.arm_dm_1.direction = MOTOR_CW;
  motor_params.arm_dm_1.max_value_ecd = 8192;
  
  motor_params.arm_dm_2.reduction_ratio = 10;
  motor_params.arm_dm_2.output_radius = 1;  
  motor_params.arm_dm_2.direction = MOTOR_CW;
  motor_params.arm_dm_2.max_value_ecd = 8192;
  
  	//MG�������
  motor_params.arm_mg_1.reduction_ratio = 6;
  motor_params.arm_mg_1.output_radius = 1;  
  motor_params.arm_mg_1.direction = MOTOR_CW;
  motor_params.arm_mg_1.max_value_ecd = 216000;
  motor_params.arm_mg_1.offset_ecd = 0;
  
  
//  //M15�������
//  motor_params.arm_motor_bottom.reduction_ratio = 1;
//  motor_params.arm_motor_bottom.output_radius = 1;  
//  motor_params.arm_motor_bottom.direction = MOTOR_CW;
//  motor_params.arm_motor_bottom.max_value_ecd = 8192;
//  motor_params.arm_motor_bottom.offset_ecd = 0; 

  //�����������
//  motor_params.arm_motor_bottom.reduction_ratio = 6;
//  motor_params.arm_motor_bottom.output_radius = 1;  
//  motor_params.arm_motor_bottom.direction = MOTOR_CW;
//  motor_params.arm_motor_bottom.max_value_ecd = 32767;
//  motor_params.arm_motor_bottom.offset_ecd = 0;
//  
//  motor_params.arm_motor_middle.reduction_ratio = 6;
//  motor_params.arm_motor_middle.output_radius = 1;  
//  motor_params.arm_motor_middle.direction = MOTOR_CW;
//  motor_params.arm_motor_middle.max_value_ecd = 32767;
//  motor_params.arm_motor_middle.offset_ecd = 0;
//  
//  motor_params.arm_motor_top_roll.reduction_ratio = 6;
//  motor_params.arm_motor_top_roll.output_radius = 1;  
//  motor_params.arm_motor_top_roll.direction = MOTOR_CW;
//  motor_params.arm_motor_top_roll.max_value_ecd = 32767;
//  motor_params.arm_motor_top_roll.offset_ecd = 0;
//  
//  motor_params.arm_motor_top_pitch.reduction_ratio = 6;
//  motor_params.arm_motor_top_pitch.output_radius = 1;  
//  motor_params.arm_motor_top_pitch.direction = MOTOR_CW;
//  motor_params.arm_motor_top_pitch.max_value_ecd = 32767;
//  motor_params.arm_motor_top_pitch.offset_ecd = 0;
  
  

  /**************  �������  **************/



  /************** ���PID���� **************/



  // �����ĸ����PID����PID����
  motor_params.chassis_motor_1_ang_vel.type_selection = PID_DELTA;
  motor_params.chassis_motor_1_ang_vel.kp = 10000;
  motor_params.chassis_motor_1_ang_vel.ki = 200000;
  motor_params.chassis_motor_1_ang_vel.kd_fb = 0;
  motor_params.chassis_motor_1_ang_vel.kd_ex = 0;
  motor_params.chassis_motor_1_ang_vel.k_ff = 0;
  motor_params.chassis_motor_1_ang_vel.max_out_value = 10000;
  motor_params.chassis_motor_1_ang_vel.min_out_value = -10000;
  motor_params.chassis_motor_1_ang_vel.limit_output = true;
  motor_params.chassis_motor_1_ang_vel.max_integral = 0;
  motor_params.chassis_motor_1_ang_vel.min_integral = -0;
  motor_params.chassis_motor_1_ang_vel.limit_integral = true;
  motor_params.chassis_motor_1_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.chassis_motor_1_ang_vel.kd_able_error_range = 0;
  motor_params.chassis_motor_1_ang_vel.ki_able_error_range = 0;

  motor_params.chassis_motor_2_ang_vel = motor_params.chassis_motor_1_ang_vel;
  motor_params.chassis_motor_3_ang_vel = motor_params.chassis_motor_1_ang_vel;
  motor_params.chassis_motor_4_ang_vel = motor_params.chassis_motor_1_ang_vel;

  //��е��3508�����PID����
  //���ٶȻ�PID����
  motor_params.arm_3508_ang_vel.type_selection = PID_DELTA;
  motor_params.arm_3508_ang_vel.kp = 900;
  motor_params.arm_3508_ang_vel.ki = 20000;
  motor_params.arm_3508_ang_vel.kd_fb = 0;
  motor_params.arm_3508_ang_vel.kd_ex = 0;
  motor_params.arm_3508_ang_vel.k_ff = 0;
  motor_params.arm_3508_ang_vel.max_out_value = 10000;
  motor_params.arm_3508_ang_vel.min_out_value = -10000;
  motor_params.arm_3508_ang_vel.limit_output = true;
  motor_params.arm_3508_ang_vel.max_integral = 0.1;
  motor_params.arm_3508_ang_vel.min_integral = -0.1;
  motor_params.arm_3508_ang_vel.limit_integral = true;
  motor_params.arm_3508_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.arm_3508_ang_vel.kd_able_error_range = 0;
  motor_params.arm_3508_ang_vel.ki_able_error_range = 0;
  
  //�ǶȻ�PID����
  motor_params.arm_3508_ang.type_selection = PID_ABSOLUTE;
  motor_params.arm_3508_ang.kp = 5;
  motor_params.arm_3508_ang.ki = 0;
  motor_params.arm_3508_ang.kd_fb = 0;
  motor_params.arm_3508_ang.kd_ex = 0;
  motor_params.arm_3508_ang.k_ff = 0;
  motor_params.arm_3508_ang.max_out_value = 10000;
  motor_params.arm_3508_ang.min_out_value = -10000;
  motor_params.arm_3508_ang.limit_output = true;
  motor_params.arm_3508_ang.max_integral = 0.1;
  motor_params.arm_3508_ang.min_integral = -0.1;
  motor_params.arm_3508_ang.limit_integral = true;
  motor_params.arm_3508_ang.add = &cmt::simple_adder_instance_f;
  motor_params.arm_3508_ang.kd_able_error_range = 0;
  motor_params.arm_3508_ang.ki_able_error_range = 0;    

  //��6020�����PID����
  //���ٶȻ�PID����
  motor_params.gimbal_6020_ang_vel.type_selection = PID_DELTA;
  motor_params.gimbal_6020_ang_vel.kp = 200;
  motor_params.gimbal_6020_ang_vel.ki = 20000;
//	motor_params.arm_6020_ang_vel.kp = 200;
//  motor_params.arm_6020_ang_vel.ki = 30000;
  motor_params.gimbal_6020_ang_vel.kd_fb = 0;
  motor_params.gimbal_6020_ang_vel.kd_ex = 0;
  motor_params.gimbal_6020_ang_vel.k_ff = 0;
  motor_params.gimbal_6020_ang_vel.max_out_value = 10000;
  motor_params.gimbal_6020_ang_vel.min_out_value = -10000;
  motor_params.gimbal_6020_ang_vel.limit_output = true;
  motor_params.gimbal_6020_ang_vel.max_integral = 0.1;
  motor_params.gimbal_6020_ang_vel.min_integral = -0.1;
  motor_params.gimbal_6020_ang_vel.limit_integral = true;
  motor_params.gimbal_6020_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.gimbal_6020_ang_vel.kd_able_error_range = 0;
  motor_params.gimbal_6020_ang_vel.ki_able_error_range = 0;
  
  //�ǶȻ�PID����
  motor_params.gimbal_6020_ang.type_selection = PID_ABSOLUTE;
  motor_params.gimbal_6020_ang.kp = 20;
  motor_params.gimbal_6020_ang.ki = 0;
//	motor_params.arm_6020_ang.kp = 100;
//  motor_params.arm_6020_ang.ki = 0;
  motor_params.gimbal_6020_ang.kd_fb = 0;
  motor_params.gimbal_6020_ang.kd_ex = 0;
  motor_params.gimbal_6020_ang.k_ff = 0;
  motor_params.gimbal_6020_ang.max_out_value = 10000;
  motor_params.gimbal_6020_ang.min_out_value = -10000;
  motor_params.gimbal_6020_ang.limit_output = true;
  motor_params.gimbal_6020_ang.max_integral = 0.1;
  motor_params.gimbal_6020_ang.min_integral = -0.1;
  motor_params.gimbal_6020_ang.limit_integral = true;
  motor_params.gimbal_6020_ang.add = &cmt::simple_adder_instance_f;
  motor_params.gimbal_6020_ang.kd_able_error_range = 0;
  motor_params.gimbal_6020_ang.ki_able_error_range = 0;  
  
  //����3508�����PID����
  //���1
  //���ٶȻ�PID����
  motor_params.overturn1_3508_ang_vel.type_selection = PID_DELTA;
  motor_params.overturn1_3508_ang_vel.kp = 500;
  motor_params.overturn1_3508_ang_vel.ki = 10000;
  motor_params.overturn1_3508_ang_vel.kd_fb = 0;
  motor_params.overturn1_3508_ang_vel.kd_ex = 0;
  motor_params.overturn1_3508_ang_vel.k_ff = 0;
  motor_params.overturn1_3508_ang_vel.max_out_value = 10000;
  motor_params.overturn1_3508_ang_vel.min_out_value = -10000;
  motor_params.overturn1_3508_ang_vel.limit_output = true;
  motor_params.overturn1_3508_ang_vel.max_integral = 0.1;
  motor_params.overturn1_3508_ang_vel.min_integral = -0.1;
  motor_params.overturn1_3508_ang_vel.limit_integral = true;
  motor_params.overturn1_3508_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.overturn1_3508_ang_vel.kd_able_error_range = 0;
  motor_params.overturn1_3508_ang_vel.ki_able_error_range = 0;
  
  //�ǶȻ�PID����
  motor_params.overturn1_3508_ang.type_selection = PID_ABSOLUTE;
  motor_params.overturn1_3508_ang.kp = 5;
  motor_params.overturn1_3508_ang.ki = 10;
  motor_params.overturn1_3508_ang.kd_fb = 0;
  motor_params.overturn1_3508_ang.kd_ex = 0;
  motor_params.overturn1_3508_ang.k_ff = 0;
  motor_params.overturn1_3508_ang.max_out_value = 10000;
  motor_params.overturn1_3508_ang.min_out_value = -10000;
  motor_params.overturn1_3508_ang.limit_output = true;
  motor_params.overturn1_3508_ang.max_integral = 0.1;
  motor_params.overturn1_3508_ang.min_integral = -0.1;
  motor_params.overturn1_3508_ang.limit_integral = true;
  motor_params.overturn1_3508_ang.add = &cmt::simple_adder_instance_f;
  motor_params.overturn1_3508_ang.kd_able_error_range = 0;
  motor_params.overturn1_3508_ang.ki_able_error_range = 0;    
  
    //���2
  //���ٶȻ�PID����
  motor_params.overturn2_3508_ang_vel.type_selection = PID_DELTA;
  motor_params.overturn2_3508_ang_vel.kp = 500;
  motor_params.overturn2_3508_ang_vel.ki = 10000;
  motor_params.overturn2_3508_ang_vel.kd_fb = 0;
  motor_params.overturn2_3508_ang_vel.kd_ex = 0;
  motor_params.overturn2_3508_ang_vel.k_ff = 0;
  motor_params.overturn2_3508_ang_vel.max_out_value = 10000;
  motor_params.overturn2_3508_ang_vel.min_out_value = -10000;
  motor_params.overturn2_3508_ang_vel.limit_output = true;
  motor_params.overturn2_3508_ang_vel.max_integral = 0.1;
  motor_params.overturn2_3508_ang_vel.min_integral = -0.1;
  motor_params.overturn2_3508_ang_vel.limit_integral = true;
  motor_params.overturn2_3508_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.overturn2_3508_ang_vel.kd_able_error_range = 0;
  motor_params.overturn2_3508_ang_vel.ki_able_error_range = 0;
  
  //�ǶȻ�PID����
  motor_params.overturn2_3508_ang.type_selection = PID_ABSOLUTE;
  motor_params.overturn2_3508_ang.kp = 5;
  motor_params.overturn2_3508_ang.ki = 10;
  motor_params.overturn2_3508_ang.kd_fb = 0;
  motor_params.overturn2_3508_ang.kd_ex = 0;
  motor_params.overturn2_3508_ang.k_ff = 0;
  motor_params.overturn2_3508_ang.max_out_value = 16000;
  motor_params.overturn2_3508_ang.min_out_value = -16000;
  motor_params.overturn2_3508_ang.limit_output = true;
  motor_params.overturn2_3508_ang.max_integral = 0.1;
  motor_params.overturn2_3508_ang.min_integral = -0.1;
  motor_params.overturn2_3508_ang.limit_integral = true;
  motor_params.overturn2_3508_ang.add = &cmt::simple_adder_instance_f;
  motor_params.overturn2_3508_ang.kd_able_error_range = 0;
  motor_params.overturn2_3508_ang.ki_able_error_range = 0; 
  
  /************** ���PID���� **************/


  /************** ����Ƶ�� **************/


  // �ĸ����̵�����������ݻ�ȡ������PID��������
  motor_params.chassis_motor_1.interval = 1e6f / 200.0f;
  motor_params.chassis_motor_1_ang_vel.interval = 1e6f / 200.0f;

  motor_params.chassis_motor_2.interval = 1e6f / 200.0f;
  motor_params.chassis_motor_2_ang_vel.interval = 1e6f / 200.0f;

  motor_params.chassis_motor_3.interval = 1e6f / 200.0f;
  motor_params.chassis_motor_3_ang_vel.interval = 1e6f / 200.0f;

  motor_params.chassis_motor_4.interval = 1e6f / 200.0f;
  motor_params.chassis_motor_4_ang_vel.interval = 1e6f / 200.0f;
  
  //�ĸ���е�۵�����������ݻ�ȡ
  
  
  //��е��3508�����ȡ������pid��������
  	motor_params.arm_3508.interval =  1e6f / 200.0f;
   motor_params.arm_3508_ang.interval = 1e6f / 200.0f;
   motor_params.arm_3508_ang_vel.interval = 1e6f / 200.0f;
	
   //��е��6020�����ȡƵ��
   motor_params.gimbal_6020.interval =  1e6f / 200.0f;
   motor_params.gimbal_6020_ang.interval = 1e6f / 200.0f;
   motor_params.gimbal_6020_ang_vel.interval = 1e6f / 200.0f;
  
   //����3508�����ȡƵ��
   motor_params.overturn1_3508.interval =  1e6f / 200.0f;
   motor_params.overturn1_3508_ang.interval = 1e6f / 200.0f;
   motor_params.overturn1_3508_ang_vel.interval = 1e6f / 200.0f;	
	
	motor_params.overturn2_3508.interval =  1e6f / 200.0f;
   motor_params.overturn2_3508_ang.interval = 1e6f / 200.0f;
   motor_params.overturn2_3508_ang_vel.interval = 1e6f / 200.0f;
	
  //mg�����ȡ����Ƶ��
  	motor_params.arm_mg_1.interval =  1e6f / 200.0f;
	
  //DM�����ȡƵ��
   motor_params.arm_dm_1.interval =  1e6f / 200.0f;
   motor_params.arm_dm_2.interval =  1e6f / 200.0f;  
	
  
  // ��������
  motor_params.control_tasks_interval.chassis_task_interval = 1e6f / 200.0f;
  
  //��е������
  motor_params.control_tasks_interval.arm_task_interval = 1e6f / 200.0f;
  
    //��̨����
  motor_params.control_tasks_interval.gimbal_task_interval = 1e6f / 200.0f;
  
  // LED ����
  motor_params.control_tasks_interval.led_task_interval = 1e6f / 200.0f;

  // ����ϵͳ���ݴ�������
  motor_params.control_tasks_interval.referee_system_task_interval = 1e6f / 100.0f;

  // CAN1 0x1FF��ַ��������
  motor_params.control_tasks_interval.can1_send_0x1ff_task_interval = 1e6f / 200.0f;

  // CAN2 0x1FF��ַ��������
  motor_params.control_tasks_interval.can2_send_0x1ff_task_interval = 1e6f / 200.0f;
  
  // CAN1 0x200��ַ��������
  motor_params.control_tasks_interval.can1_send_0x200_task_interval = 1e6f / 200.0f;

  // CAN2 0x200��ַ��������
  motor_params.control_tasks_interval.can2_send_0x200_task_interval = 1e6f / 200.0f;
	
  
     // CAN1 0x101��ַ��������
  motor_params.control_tasks_interval.can1_send_0x101_task_interval = 1e6f / 200.0f;
  
     // CAN1 0x102��ַ��������
  motor_params.control_tasks_interval.can1_send_0x102_task_interval = 1e6f / 200.0f;
    
     // CAN1 0x32��ַ��������
  motor_params.control_tasks_interval.can1_send_0x32_task_interval = 1e6f / 200.0f;

	// CAN1 0x145��ַ��������
  motor_params.control_tasks_interval.can1_send_0x145_task_interval = 1e6f / 100.0f;
  
  /************** ����Ƶ�� **************/


}

