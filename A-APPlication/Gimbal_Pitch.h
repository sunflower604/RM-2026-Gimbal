#ifndef __GIMBAL_PITCH_H
#define __GIMBAL_PITCH_H

#include "PID.h"
#include "CAN_receive.h"
#include "remote_control.h"

void Gimbal_Pitch_Init(void);
void Gimbal_Pitch_Control(void);
#endif // __GIMBAL_PITCH_H
