#include "Gimbal_Yaw_Big.h"

PID_PositionInitTypedef BigYaw_PositionPID;
PID_PositionInitTypedef BigYaw_SpeedPID;
extern BMI088_Init_typedef BMI088_Data;					
extern BMI088_Init_typedef Can_BMI088_Data;			
extern BMI088_Init_typedef BigYaw_BMI088_Data;		
extern BMI088_Init_typedef SmallYaw_BMI088_Data;	
extern M6020_Motor Can1_M6020_MotorStatus[7];//GM6020电机状态数组
extern M6020_Motor Can2_M6020_MotorStatus[7];//GM6020电机状态数组
extern RC_ctrl_t *local_rc_ctrl;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;



void Gimbal_YawBig_Init(void)
{
	PID_PositionStructureInit (&BigYaw_PositionPID,3944);        //外环电机位置环
  PID_PositionSetParameter  (&BigYaw_PositionPID,0.5,0,0);
	PID_PositionSetEkRange		(&BigYaw_PositionPID,-50,50);		//位置式PID设置误差为0阈值
  PID_PositionSetOUTRange   (&BigYaw_PositionPID,-20000,20000);
  // PID_PositionSetNeedValueRange(&BigYaw_PositionPID,4848,0);

	PID_PositionStructureInit (&BigYaw_SpeedPID,0);              //内环速度环
  PID_PositionSetParameter  (&BigYaw_SpeedPID,45,0,0);
	PID_PositionSetEkRange		(&BigYaw_PositionPID,-2,2);
  PID_PositionSetOUTRange   (&BigYaw_SpeedPID,-15000,15000);
}

void Gimbal_YawBig_Control(void)
{
    // ============速度环pid调节代码============
//  static uint32_t tick = 0;
//   static int i = 0;
//   float target_speed = 0.0f;

//   // 每 1000ms 切换一次状态（1秒）
//   if (HAL_GetTick() - tick > 1000) {
//       tick = HAL_GetTick();
//       i++;
//   }

//   // i=0: 0 RPM, i=1: +100, i=2: 0, i=3: -100, 然后循环
//   switch (i % 4) {
//       case 0: target_speed = 0.0f;    break;   // 停
//       case 1: target_speed = 60.0f;  break;   // 正转
//       case 2: target_speed = 0.0f;    break;   // 停
//       case 3: target_speed = 60.0f; break;   // 反转
//   }
//  
//  
//   PID_PositionSetNeedValue(&BigYaw_SpeedPID, target_speed);
//   PID_PositionCalc(&BigYaw_SpeedPID, Can2_M6020_MotorStatus[0].Speed);
//  
////    Motor_6020_Voltage1((int16_t)BigYaw_SpeedPID.OUT, 0, 0, 0, &hcan2);
    // ========================

	
	
	
	
//	
//		(Can2_M6020_MotorStatus[0].ANgle - 104) = BigYaw_BMI088_Data.Yaw
		
    // ============ 1. 更新位置目标（仅打杆时）============
		BigYaw_PositionPID.Need_Value -= 0.012f * (Can2_M6020_MotorStatus[1].Position-2432);
//    BigYaw_PositionPID.Need_Value -= 0.025f * local_rc_ctrl->rc.ch[4];

			if (BigYaw_PositionPID.Need_Value > 8191.0f)
					BigYaw_PositionPID.Need_Value -= 8192.0f;
			else if (BigYaw_PositionPID.Need_Value < 0.0f)
					BigYaw_PositionPID.Need_Value += 8192.0f;
			

    // ============ 2. 位置环计算 =========================
		PID_PositionCalc_Encoder(&BigYaw_PositionPID, Can2_M6020_MotorStatus[0].Angle);
    // ===================================================

    // ============ 3. 速度环计算 =========================
    PID_PositionSetNeedValue(&BigYaw_SpeedPID, BigYaw_PositionPID.OUT + 20);
    PID_PositionCalc				(&BigYaw_SpeedPID, Can2_M6020_MotorStatus[0].Speed);
    // ===================================================

    // ============ 4. 发送输出 ===========================
//    Motor_6020_Voltage1((int16_t)BigYaw_SpeedPID.OUT, 0, 0, 0, &hcan2);
	
}
