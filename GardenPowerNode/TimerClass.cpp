#include "TimerClass.h"

TimerClass::TimerClass(){
	this->interval_millis = 1000;
}

TimerClass::TimerClass(unsigned long interval_millis){
	this->interval_millis = interval_millis;
}

void TimerClass::setInterval(unsigned long interval_millis){
	this->reset();
	this->interval_millis = interval_millis;
}

unsigned long TimerClass::getInterval(){
	return this->interval_millis; 
}

uint8_t TimerClass::check(){

  unsigned long now = millis();
  
  if ( interval_millis == 0 ){
	previous_millis = now;
	return 1;
  }
 
  if ( (now - previous_millis) >= interval_millis) {
	previous_millis += interval_millis ; 
	return 1;
  }
  
  return 0;
}

void TimerClass::reset() {
  this->previous_millis = millis();
}
