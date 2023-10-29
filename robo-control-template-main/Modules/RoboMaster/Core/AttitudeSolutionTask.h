#ifndef __ATTITUDESOLUTIONTASK_H__
#define __ATTITUDESOLUTIONTASK_H__

#include "Task.h"
#include "InertialSensor.h"
#include "stdint.h"
#include "AHRS.h"
#include "MahonyAHRS.h"
#include "PID_Controller.h"

class Robot;

// ÍÓÂÝÒÇ¿ØÎÂ
class PID_GyrotempTask : public Task_Base
{
public:
  friend class AttitudeSolutionTask;
  PID_GyrotempTask(Robot &robot0, timeus_t interval_tick_us0 = 1e6f / 5.0f) : Task_Base(robot0) // 5 Hz
  {
    this->interval_tick_us = interval_tick_us0;
  }

  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);

protected:
  PID_Controller pid_controller;
  float bmi088_temperature;
};


class AttitudeSolutionTask : public Task_Base
{
public:
  friend class PID_GyrotempTask;
  friend class Robot;
  AttitudeSolutionTask(Robot &robot0) : Task_Base(robot0)
  {
    inited = true;
    pid_gyrotemp_task_p = new PID_GyrotempTask(robot, 1e6f / 5.0f); // ´´½¨PID¿ØÎÂIMUÈÎÎñ£¬5Hz
  }

  virtual ~AttitudeSolutionTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

  PID_GyrotempTask *getPIDGyrotempTaskPointer(void)
  {
    return pid_gyrotemp_task_p;
  }
  
  Quaternion getIMUQuat(void) const
  {
    return ahrs.getIMUQuat();
  }
  
  Vector3f getIMUEuler() const
  {
    return ahrs.getIMUEuler();
  }
  Matrix3f getIMUCoord() const
  {
    return ahrs.getIMUCoordinateSystem();
  }

protected:
  MahonyAHRS ahrs;
  PID_GyrotempTask *pid_gyrotemp_task_p;
};

#endif
