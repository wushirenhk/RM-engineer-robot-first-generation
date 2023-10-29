#ifndef __RCPROTOCOL_BACKEND_H__
#define __RCPROTOCOL_BACKEND_H__

#include "Scheduler_Common.h"
#include "stdint.h"
#include "RCProtocol.h"

class RCProtocol_Backend // 抽象类
{
  friend class RCProtocol_DBUS;
public:
  RCProtocol_Backend(RCProtocol &rc_protocol);
  virtual void init(void) = 0;
  virtual bool processByte(volatile const uint8_t *buf) = 0;
  virtual void uninit(void) = 0;
  void publishRCData(RC_Data rc_data);

protected:
  RCProtocol &rc_protocol; // 引用变量
  RC_Data rc_data;
};

#endif
