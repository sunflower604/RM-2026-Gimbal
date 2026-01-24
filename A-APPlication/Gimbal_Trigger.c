#include "Gimbal_Trigger.h"

PID_PositionInitTypedef Trigger_SpeedPID;
PID_PositionInitTypedef Trigger_PositionPID;
extern motor_measure_t motor_chassis[8];
extern RC_ctrl_t *local_rc_ctrl;

void Gimbal_Trigger_Init()
{
	PID_PositionStructureInit (&Trigger_SpeedPID,100);              //拨弹盘速度环
  PID_PositionSetParameter  (&Trigger_SpeedPID,100,0,0);
  PID_PositionSetOUTRange   (&Trigger_SpeedPID,-10000,10000);
  PID_PositionSetEkRange    (&Trigger_SpeedPID, -3.0f, 3.0f);
	
}


void Gimbal_Trigger_Control()
{
	if(local_rc_ctrl->rc.s[1] == 0x02)
	PID_PositionSetNeedValue(&Trigger_SpeedPID, 3000);
	else
		PID_PositionSetNeedValue(&Trigger_SpeedPID, 0);
		
	PID_PositionCalc(&Trigger_SpeedPID, motor_chassis[6].speed_rpm);
	CAN_cmd_gimbal2(
		0,
		0,
		(int16_t)Trigger_SpeedPID.OUT,
		0
	);
}


