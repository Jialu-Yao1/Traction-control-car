#ifndef __PID_H
#define __PID_H


typedef struct 
{
	float target_val;
	float actual_val;
	float err;
	float err_last;
	float err_sum;
	float Kp,Ki,Kd;
	
} tPid;


float P_realize(tPid * pid,float actual_val);
void PID_init(void);
float PI_realize(tPid * pid,float actual_val);
float PID_realize(tPid * pid,float actual_val);
#endif
