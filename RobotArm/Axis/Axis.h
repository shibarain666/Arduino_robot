/* 2 axis robot arm's PID class */

#ifndef AXIS_H 
#define AXIS_H 

#include "Arduino.h"

class Axis{
  private:
    double kp, ki, kd;
    double P, I, D;
    double pid;
    double upper_limit, lower_limit;
    double lastSample, error;
    double sample, setpoint;
    
  public:
    Axis(double _kp, double _ki, double _kd, double _upper_limit, double _lower_limit);
    double process();
    void addNewSample(double _sample);
    void setSetPoint(double _setPoint);
};

Axis::Axis(double _kp, double _ki, double _kd, double _upper_limit, double _lower_limit){
	kp = _kp;
	ki = _ki;
	kd = _kd;
	upper_limit = _upper_limit;
	lower_limit = _lower_limit;
}

double Axis::process(){
  double deltatime = 0.01;
  error = setpoint - sample;
  P = error * kp;
  I = I + (error * ki) * deltatime;
  //D = (lastSample - sample) * Kd / (deltatime);
  lastSample = sample;
  pid = P + I + D;
  if (pid <= lower_limit) {
    pid = lower_limit;
  }
  else if (pid >= upper_limit) {
    pid = upper_limit;
  }
  return pid;
}

inline void Axis::addNewSample(double _sample) {
  sample = _sample;
}
inline void Axis::setSetPoint(double _setPoint) {
  setpoint = _setPoint;
}
  
#endif  
