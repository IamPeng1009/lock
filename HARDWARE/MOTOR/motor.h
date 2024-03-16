#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "sys.h"

#define MOTOR_IN1		PFout(12)
#define MOTOR_IN2		PDout(4)
#define MOTOR_IN3		PDout(14)
#define MOTOR_IN4		PDout(0)

extern void motor_init(void);
extern void motor_corotation_open(void);
extern void motor_corotation_close(void);

#endif


