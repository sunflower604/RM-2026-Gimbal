#include "Gimbal_Pitch.h"
#define PITCH_MID 2424          //小yaw轴中位值
#define PITCH_LEFT 1650          //小yaw轴左侧最大偏移
#define PITCH_RIGHT 1650          //小yaw轴右侧最大偏移（顺时针减小）

PID_PositionInitTypedef Pitch_PositionPID;
PID_PositionInitTypedef Pitch_SpeedPID;
extern motor_measure_t motor_chassis[7];
extern RC_ctrl_t *local_rc_ctrl;



void Gimbal_Pitch_Init(void)
{
	PID_PositionStructureInit (&Pitch_PositionPID,2424);        //外环位置环
  PID_PositionSetParameter  (&Pitch_PositionPID,1,0,0);
  PID_PositionSetOUTRange   (&Pitch_PositionPID,-400,400);
  // PID_PositionSetNeedValueRange(&Pitch_PositionPID,4848,0);

	PID_PositionStructureInit (&Pitch_SpeedPID,0);              //内环速度环
  PID_PositionSetParameter  (&Pitch_SpeedPID,100,0,0);
  PID_PositionSetOUTRange   (&Pitch_SpeedPID,-20000,20000);
  PID_PositionSetEkRange    (&Pitch_SpeedPID, -3.0f, 3.0f);
}

void Gimbal_Pitch_Control(void)
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
  //       case 1: target_speed = 100.0f;  break;   // 正转
  //       case 2: target_speed = 0.0f;    break;   // 停
  //       case 3: target_speed = 150.0f; break;   // 反转
  //   }
    
    
  //   PID_PositionSetNeedValue(&Pitch_SpeedPID, target_speed);
  //   PID_PositionCalc(&Pitch_SpeedPID, motor_chassis[5].speed_rpm);
    
  //   CAN_cmd_gimbal(0, (int16_t)Pitch_SpeedPID.OUT, 0, 0);
    // ========================












    #define RC_DEADBAND 10
    int16_t ch0 = local_rc_ctrl->rc.ch[1];

    // ============ 1. 更新位置目标（仅打杆时）============
    if (ch0 > RC_DEADBAND || ch0 < -RC_DEADBAND) {
        Pitch_PositionPID.Need_Value -= 0.025f * ch0;

        // 限幅 [0, 4848]
        if (Pitch_PositionPID.Need_Value > 4848.0f)
            Pitch_PositionPID.Need_Value = 4848.0f;
        else if (Pitch_PositionPID.Need_Value < 0.0f)
            Pitch_PositionPID.Need_Value = 0.0f;
    }
    // ===================================================

    // ============ 2. 位置环计算 =========================
    PID_PositionCalc(&Pitch_PositionPID, motor_chassis[5].ecd);
    // ===================================================

    // ============ 3. 关键修复：死区内强制速度目标为 0 =====
    float target_speed = 0.0f;
    if (ch0 > RC_DEADBAND || ch0 < -RC_DEADBAND) {
        target_speed = Pitch_PositionPID.OUT; // 打杆时用位置环输出
    }
    // 松手时 target_speed = 0 → 速度环无指令 → 电机静音
    // ===================================================

    // ============ 4. 速度环计算 =========================
    PID_PositionSetNeedValue(&Pitch_SpeedPID, target_speed);
    PID_PositionCalc(&Pitch_SpeedPID, motor_chassis[5].speed_rpm);
    // ===================================================

    // ============ 5. 发送输出 ===========================
    CAN_cmd_gimbal(0, (int16_t)Pitch_SpeedPID.OUT, 0, 0);
}


