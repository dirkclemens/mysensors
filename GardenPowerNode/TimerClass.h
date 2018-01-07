#pragma once

#ifndef TimerClass_h
#define TimerClass_h

#include "Arduino.h"

class TimerClass {
  public:
	TimerClass();
	TimerClass(unsigned long interval_millis);
	void setInterval(unsigned long interval_millis);
	unsigned long getInterval();
	uint8_t check();
	void reset();

  private:
	unsigned long  previous_millis, interval_millis;
};

#endif
