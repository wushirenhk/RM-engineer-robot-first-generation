#ifndef PIDCONTROLTASK_H
#define PIDCONTROLTASK_H

#include "Task.h"
#include "stdint.h"
#include "PID_Controller.h"

class Robot;

class PIDControlTask : public Task_Base
{
public:
  friend class PIDMotorControlTask;
  friend class Motor_RM_PIDControlTask;
  friend class Motor_MC_ControlTask;
  friend class MainControlTask;
  friend class Robot;
  PIDControlTask(Robot &robot0, timeus_t interval_tick_us0 = 0) : Task_Base(robot0)
  {
    this->interval_tick_us = interval_tick_us0;
  }

  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);

  // 设置PID参数结构体引用
  void setPIDControllerParams(const PID_Params_t &params)
  {
    pid_controller.setParams(params);
  }
  // 设置PID期望值
  void setPIDControllerExpect(float expect)
  {
    pid_controller.setExpect(expect);
  }
  // 设置PID反馈值
  void setPIDControllerFeedback(float feedback)
  {
    pid_controller.setFeedback(feedback);
  }
  // 设置PID前馈值
  void setPIDControllerFeedforward(float feedforward)
  {
    pid_controller.setFeedforward(feedforward);
  }
  // 设置PID输入值
  void setPIDControllerInput(float expect, float feedback, float feedforward)
  {
    pid_controller.setInput(expect, feedback, feedforward);
  }
  float getOutput()
  {
    return pid_controller.getOutput();
  }
  float getLimitedOutput()
  {
    return pid_controller.getLimitedOutput();
  }
  void limitOutput(){
    pid_controller.limitOutput();
  }
  void clearPID()
  {
    pid_controller.clear();
  }
  // 计算PID
  void calculatePID(timeus_t _dT_us,           //周期（us）
                    float _expect,			       //期望值
                    float _feedback,           //反馈值
                    float _feedforward = 0.0f  //前馈值
                   )
  {
    pid_controller.calculateWithCPU(_dT_us, _expect, _feedback, _feedforward);
  }
  // 计算PID
  void calculatePID(timeus_t _dT_us)     //周期（us）
  {
    pid_controller.calculateWithCPU(_dT_us);
//    pid_controller.calculateWithCPU(_dT_us,
//                             pid_controller.getPidData().expect,
//                             pid_controller.getPidData().feedback,
//                             pid_controller.getPidData().feedforward
//                            );
  }
	void printPID(void);

protected:
  PID_Controller pid_controller;
};


#endif