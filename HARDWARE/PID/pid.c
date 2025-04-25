#include "pid.h"
tPid pidMotor1Speed;
tPid pidMotor2Speed;
tPid pidHW_Tracking;



tPid pidFollow;   
tPid pidMPU6050YawMovement;  


void PID_init()
{
	pidMotor1Speed.actual_val=0.0;
	pidMotor1Speed.target_val=0.00;
	pidMotor1Speed.err=0.0;
	pidMotor1Speed.err_last=0.0;
	pidMotor1Speed.err_sum=0.0;
	pidMotor1Speed.Kp=15;
	pidMotor1Speed.Ki=5;
	pidMotor1Speed.Kd=0;
	
	pidMotor2Speed.actual_val=0.0;
	pidMotor2Speed.target_val=0.00;
	pidMotor2Speed.err=0.0;
	pidMotor2Speed.err_last=0.0;
	pidMotor2Speed.err_sum=0.0;
	pidMotor2Speed.Kp=15;
	pidMotor2Speed.Ki=5;
	pidMotor2Speed.Kd=0;
	
	pidHW_Tracking.actual_val=0.0;
	pidHW_Tracking.target_val=0.00;
	pidHW_Tracking.err=0.0;
	pidHW_Tracking.err_last=0.0;
	pidHW_Tracking.err_sum=0.0;
	pidHW_Tracking.Kp=-1.50;
	pidHW_Tracking.Ki=0;
	pidHW_Tracking.Kd=0.80;
	

	
	
	
	pidFollow.actual_val=0.0;
	pidFollow.target_val=22.50;
	pidFollow.err=0.0;
	pidFollow.err_last=0.0;
	pidFollow.err_sum=0.0;
	pidFollow.Kp=-0.5;
	pidFollow.Ki=-0.001;
	pidFollow.Kd=0;
	
	pidMPU6050YawMovement.actual_val=0.0;
	pidMPU6050YawMovement.target_val=0.00;
	pidMPU6050YawMovement.err=0.0;
	pidMPU6050YawMovement.err_last=0.0;
	pidMPU6050YawMovement.err_sum=0.0;
	pidMPU6050YawMovement.Kp=0.02;
	pidMPU6050YawMovement.Ki=0;
	pidMPU6050YawMovement.Kd=0.1;
	
}

float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}

float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->err_sum += pid->err;

	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
	
	return pid->actual_val;
}

float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->err_sum += pid->err;
	
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

