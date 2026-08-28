#ifndef __PID_H
#define __PID_H

typedef struct {		
	float Target;		
	float Actual;		
	float Actual1;		
	float Out;			
	
	float Kp;			
	float Ki;			
	float Kd;			
	
	float Error0;		
	float Error1;		
	float ErrorInt;		
	
	float ErrorIntMax;	
	float ErrorIntMin;	
	
	float OutMax;		
	float OutMin;		
	
	float OutOffset;	

	float dt;
} balance_pid_t;

void pidInit(balance_pid_t *p);
void pidUpdate(balance_pid_t *p);

#endif
