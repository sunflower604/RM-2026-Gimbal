#include "Gimbal_Shoot.h"
#include "stm32f4xx_it.h"

PID_PositionInitTypedef ShootLeft_SpeedPID;
PID_PositionInitTypedef ShootRight_SpeedPID;
uint8_t Gimbal_Shoot_Flag;
extern M3508_Motor Can1_M3508_MotorStatus[8];
extern M3508_Motor Can2_M3508_MotorStatus[8];
extern RC_ctrl_t *local_rc_ctrl;
extern uint8_t Remote_Status;
extern uint8_t MiniPC_Flag;
extern NewRxDataStruct NewRxData;     // 解析后的数据

void Gimbal_Shoot_Init()
{
	PID_PositionStructureInit (&ShootRight_SpeedPID,0);              //左轮速度环
  PID_PositionSetParameter  (&ShootRight_SpeedPID,30,0,0);
  PID_PositionSetOUTRange   (&ShootRight_SpeedPID,-16000,16000);
  PID_PositionSetEkRange    (&ShootRight_SpeedPID, -3.0f, 3.0f);
	
	PID_PositionStructureInit (&ShootLeft_SpeedPID,0);              //左轮速度环
  PID_PositionSetParameter  (&ShootLeft_SpeedPID,30,0,0);
  PID_PositionSetOUTRange   (&ShootLeft_SpeedPID,-16000,16000);
  PID_PositionSetEkRange    (&ShootLeft_SpeedPID, -3.0f, 3.0f);
}

void Gimbal_Shoot_Control()
{
	if(Remote_Status==1 && local_rc_ctrl->rc.s[1]!=1){//遥控器手动控制
		float target_speed = 0.0f;
		if(local_rc_ctrl->rc.s[1] == 0x02)
		{
			target_speed = -6000;
			Gimbal_Shoot_Flag = 1;
		}
		else
		{
			target_speed = 0;
			Gimbal_Shoot_Flag = 0;
			
		}
		PID_PositionSetNeedValue(&ShootLeft_SpeedPID, target_speed);
		PID_PositionCalc				(&ShootLeft_SpeedPID, (float)Can1_M3508_MotorStatus[0].RotorSpeed);//ID1
		PID_PositionSetNeedValue(&ShootRight_SpeedPID,-target_speed);
		PID_PositionCalc				(&ShootRight_SpeedPID, (float)Can1_M3508_MotorStatus[1].RotorSpeed);//ID2
		
		Motor_3508_Current1(
			(int16_t)ShootLeft_SpeedPID.OUT,
			(int16_t)ShootRight_SpeedPID.OUT,
			0,
			0,
			&hcan1
		);
	}
	else if(Remote_Status==1 && MiniPC_Flag==1 && local_rc_ctrl->rc.s[1]==1){//小电脑控制
		float target_speed = 0.0f;
		if(NewRxData.numA == 0x02)
		{
			target_speed = -6000;
			Gimbal_Shoot_Flag = 1;
		}
		else
		{
			target_speed = 0;
			Gimbal_Shoot_Flag = 0;
			
		}
		PID_PositionSetNeedValue(&ShootLeft_SpeedPID, target_speed);
		PID_PositionCalc				(&ShootLeft_SpeedPID, (float)Can1_M3508_MotorStatus[0].RotorSpeed);//ID1
		PID_PositionSetNeedValue(&ShootRight_SpeedPID,-target_speed);
		PID_PositionCalc				(&ShootRight_SpeedPID, (float)Can1_M3508_MotorStatus[1].RotorSpeed);//ID2
		
		Motor_3508_Current1(
			(int16_t)ShootLeft_SpeedPID.OUT,
			(int16_t)ShootRight_SpeedPID.OUT,
			0,
			0,
			&hcan1
		);
		
	}
	else if(Remote_Status == 0){
		PID_PositionClean(&ShootRight_SpeedPID);
		PID_PositionClean(&ShootLeft_SpeedPID);
		PID_PositionSetNeedValue(&ShootRight_SpeedPID, 0);
		PID_PositionSetNeedValue(&ShootLeft_SpeedPID, 0);
		Motor_3508_Current1(
			0,
			0,
			0,
			0,
			&hcan1
		);
	}
}




