#ifndef MOTOR_CONTROL_TASKS_H
#define MOTOR_CONTROL_TASKS_H

#include "Task.h"
#include "stdint.h"
#include "PID_Controller.h"
#include "Motor.h"
#include "CanDevice.h"
#include "Motor_RM.h"
#include "CANSendTask.h"
#include "PIDControlTask.h"

class Robot;

class Motor_RM_PIDControlTask : public Task_Base
{
public:
  friend class PID_MotorAngleControlTask;
  friend class PID_MotorAngularVelocityControlTask;
  friend class GimbalControlTask;
  friend class ChassisControlTask;
  friend class Robot;
  friend class AmmoRM_ControlTask;
  friend class Arm_ControlTask;
  friend class RefereeSystemTask;

  Motor_RM_PIDControlTask(
    Robot &robot0,
    CanDevice &_can_device,
    uint16_t _can_rx_id, uint16_t _can_tx_id,
    uint8_t _can_tx_data_start_pos,
    int8_t _motor_type,
    timeus_t interval_tick_us0 = 0
  );

  virtual ~Motor_RM_PIDControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

//  PID_MotorRM_ControlTask *getPIDMotorAngleControlTaskPointer(void)
//  {
//    return pid_motor_angle_control_task_p;
//  }
  void setPIDControllerParams(PIDControlTask &task, const PID_Params_t &params)
  {
    task.setPIDControllerParams(params);
  }
  PIDControlTask* getAngleTaskPointer(){
    return angle_control_task_p;
  }
  PIDControlTask* getAngularVelocityTaskPointer(){
    return angular_velocity_control_task_p;
  }
//  void setPIDControllerInput(PID_MotorRM_ControlTask &task, float expect, float feedback, float in_ff)
//  {
//    task.pid_controller.getPIDDataPointer()->expect = expect;
//    task.pid_controller.getPIDDataPointer()->feedback = feedback;
//    task.pid_controller.getPIDDataPointer()->in_ff = in_ff;
//  }
protected:
  //Motor *motor;
  Motor_RM *motor_backend_p;
  CanDevice &can_device;
  uint8_t can_tx_data_start_pos;
  // CANSendTask *can_send_task_p;
  // CAN_Rx_Data_Pack_t can_rx_data;
  PIDControlTask *angle_control_task_p;
  PIDControlTask *angular_velocity_control_task_p;
};


#endif
