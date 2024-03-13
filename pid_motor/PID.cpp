#include "PID.h"
#include <cmath>
#include <Arduino.h>

PID::PID(float kp_, float ki_, float kd_, float minU_, float maxU_, float minD_, float maxD_, unsigned long sampleTime_){
	kp = kp_;
	ki = ki_;
	kd = kd_;
	minU = minU_;
	maxU = maxU_;
	minD = minD_;
	maxD = maxD_;
	
	sampleTime = sampleTime_;
	
	error = 0;
	accumulated_error = 0;
	previous_error = 0;
	difference_error = 0;

	u = 0;
}

int PID::pid_controller(double desired, double current){
	//Saturating desired
	desired = (desired >= maxD) ? maxD : desired;	
	desired = (desired <= minD) ? minD : desired;

	error = desired - current; // Error

	accumulated_error += error * sampleTime/1000.0; //Integrate Error Dividing by 1000 because in millis
	difference_error = (error - previous_error) / (sampleTime/1000.0); //Differentiate Error
	previous_error = error;

	//PID formula
	u = (error * kp) + (accumulated_error * ki) + (difference_error * kd); 

	//Saturating input of system
	u = (u >= maxU) ? maxU : u;
	u = (u <= minU) ? minU : u;


	return trunc(u);
}

