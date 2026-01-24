#ifndef GIMBAL_TRIGGER_H
#define GIMBAL_TRIGGER_H

#include "PID.h"
#include "CAN_receive.h"
#include "remote_control.h"

void Gimbal_Trigger_Init();
void Gimbal_Trigger_Control();

#endif //GIMBAL_TRIGGER_H
