#include "Gimbal_Shoot.h"

PID_PositionInitTypedef ShootLeft_SpeedPID;
PID_PositionInitTypedef ShootRight_SpeedPID;
extern motor_measure_t motor_chassis[7];
extern RC_ctrl_t *local_rc_ctrl;

void Gimbal_Shoot_Init()
{
	PID_PositionStructureInit (&ShootRight_SpeedPID,0);              //左轮速度环
  PID_PositionSetParameter  (&ShootRight_SpeedPID,100,0,0);
  PID_PositionSetOUTRange   (&ShootRight_SpeedPID,-20000,20000);
  PID_PositionSetEkRange    (&ShootRight_SpeedPID, -3.0f, 3.0f);
	
	PID_PositionStructureInit (&ShootLeft_SpeedPID,0);              //左轮速度环
  PID_PositionSetParameter  (&ShootLeft_SpeedPID,100,0,0);
  PID_PositionSetOUTRange   (&ShootLeft_SpeedPID,-20000,20000);
  PID_PositionSetEkRange    (&ShootLeft_SpeedPID, -3.0f, 3.0f);
}

void Gimbal_Shoot_Control()
{
	static uint32_t tick = 0;
	static int i = 0;
	float target_speed = 0.0f;
	
	// 每 1000ms 切换一次状态（1秒）
	if (HAL_GetTick() - tick > 1000) {
			tick = HAL_GetTick();
			i++;
	}
	
	// i=0: 0 RPM, i=1: +100, i=2: 0, i=3: -100, 然后循环
	switch (i % 4) {
			case 0: target_speed = 0.0f;    break;   // 停
			case 1: target_speed = 100.0f;  break;   // 正转
			case 2: target_speed = 0.0f;    break;   // 停
			case 3: target_speed = 150.0f; break;   // 反转
	}
	
	PID_PositionSetNeedValue(&ShootLeft_SpeedPID, target_speed);
	PID_PositionCalc				(&ShootLeft_SpeedPID, motor_chassis[6].speed_rpm);
	PID_PositionSetNeedValue(&ShootRight_SpeedPID, target_speed);
	PID_PositionCalc				(&ShootRight_SpeedPID, motor_chassis[7].speed_rpm);
	
	CAN_cmd_gimbal(
		0,
		0,
		(int16_t)ShootLeft_SpeedPID.OUT,
		(int16_t)ShootRight_SpeedPID.OUT
	);
}




