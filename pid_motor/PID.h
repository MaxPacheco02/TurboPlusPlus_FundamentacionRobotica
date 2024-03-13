#ifndef _PID_H_
#define _PID_H_

class PID{

	public: 
		PID(float kp_, float ki_, float kd_, float minU_, float maxU_, float minD_, float maxD_, unsigned long sampleTime_);

		// Input desired, Output U
		int pid_controller(double desired, double current);

	private:
		float kp, ki, kd, minU, maxU, minD, maxD;
		unsigned long sampleTime;
		double error, accumulated_error, previous_error, difference_error;
		float u;
};

#endif 
