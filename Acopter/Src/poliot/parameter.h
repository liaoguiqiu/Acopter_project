#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "include.h"

typedef struct
{
	float kp;
	
	float ki;
        float kd;
	float kdamp;
	 

}pid_t;
 
class PARAMERTER
{
public:


	PARAMERTER()
	{
	
	}
	void Para_Init();
	void PID_Para_Init();
private:

};
 
extern PARAMERTER parameter;
#endif

