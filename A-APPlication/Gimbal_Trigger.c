#include "Gimbal_Trigger.h"
#include "stm32f4xx_it.h"

PID_PositionInitTypedef Trigger_SpeedPID;
PID_PositionInitTypedef Trigger_PositionPID;
extern M2006_Motor Can1_M2006_MotorStatus[8];//M2006电机状态数组
extern M2006_Motor Can2_M2006_MotorStatus[8];//M2006电机状态数组
extern RC_ctrl_t *local_rc_ctrl;
extern uint8_t Gimbal_Shoot_Flag;
extern uint8_t Remote_Status; 
extern uint8_t MiniPC_Flag;
extern NewRxDataStruct NewRxData;     // 解析后的数据

void Gimbal_Trigger_Init()
{
	PID_PositionStructureInit (&Trigger_SpeedPID,0);              //拨弹盘速度环
  PID_PositionSetParameter  (&Trigger_SpeedPID,20,0,0);
  PID_PositionSetOUTRange   (&Trigger_SpeedPID,-10000,10000);
  PID_PositionSetEkRange    (&Trigger_SpeedPID, -3.0f, 3.0f);
	
}


void Gimbal_Trigger_Control()
{
	if(Remote_Status==1 && local_rc_ctrl->rc.s[1]!=1){//遥控器手动控制
		if(local_rc_ctrl->rc.ch[4] > 100 && Gimbal_Shoot_Flag )
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, 2000);
		}
		else if(local_rc_ctrl->rc.ch[4] < -100)
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, -2000);
		}
		else
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, 0);
			PID_PositionClean(&Trigger_SpeedPID);	//位置式PID清理
		}
		PID_PositionCalc(&Trigger_SpeedPID, Can1_M2006_MotorStatus[6].RotorSpeed);
		Motor_2006_Current2(0,0,(int16_t)Trigger_SpeedPID.OUT,0,&hcan1);

	}
	else if(Remote_Status==1 && MiniPC_Flag==1 && local_rc_ctrl->rc.s[1]==1){//小电脑控制
		if(NewRxData.data5 > 100 && Gimbal_Shoot_Flag )
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, 2000);
		}
		else if(NewRxData.data5 < -100)
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, -2000);
		}
		else
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, 0);
			PID_PositionClean(&Trigger_SpeedPID);	//位置式PID清理
		}
		PID_PositionCalc(&Trigger_SpeedPID, Can1_M2006_MotorStatus[6].RotorSpeed);
		Motor_2006_Current2(0,0,(int16_t)Trigger_SpeedPID.OUT,0,&hcan1);
	}
	else if(Remote_Status == 0){
		PID_PositionClean(&Trigger_SpeedPID);
		PID_PositionSetNeedValue(&Trigger_SpeedPID, 0);
		Motor_2006_Current2(0,0,0,0,&hcan1);
	}
}

