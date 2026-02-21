#include "Gimbal_Warning.h"
#include "Buzzer.h"
#include "LED.h"
#include "task.h"

int16_t music[]=
{
    // "都是勇敢的" (原: M3,M3,S,S,M1,M2,M1,M3,M3,S)
    M3, M3, S, S,           // M3,M3,S,S -> 保持不变 (4个250ms单元)
    M1, M2, M1,             // M1,M2,M1 -> 保持不变 (3个250ms单元)
    M3, M3, S, S, S, S,     // M3,M3,S -> 延长S (M3,M3,4个S -> 6个250ms单元)

    // "你额头的伤口你的不同你犯的错" (原: M1,M2,M1,M2,M3,L6,M1,L6,M1,L6,M1,M2,M1,L7,L7,S,S)
    M1, M2, M1,             // M1,M2,M1 -> 保持 (3个250ms单元)
    M2, M3, L6,             // M2,M3,L6 -> 保持 (3个250ms单元)
    M1, M1, L6, L6, M1, M1, L6, L6, // M1,L6,M1,L6 -> 每个音符拆成两个125ms (M1,M1,L6,L6,M1,M1,L6,L6 -> 8个125ms单元 = 4个250ms单元)
    M1, M2, M1,             // M1,M2,M1 -> 保持 (3个250ms单元)
    L7, L7, S, S,           // L7,L7,S,S -> 延长S (4个250ms单元)

    // "都不必隐藏" (原: M3,M3,S,S,M1,M2,M1,M3,M3,S)
    M3, M3, S, S,           // M3,M3,S,S -> 保持 (4个250ms单元)
    M1, M2, M1,             // M1,M2,M1 -> 保持 (3个250ms单元)
    M3, M3, S, S, S, S,     // M3,M3,S -> 延长S (6个250ms单元)

    // "你破旧的玩偶你的面具你的自我" (原: M1,M2,M1,M2,M3,L6,M1,L6,M1,L6,M1,M3,M2,L7,L7,S,S)
    M1, M2, M1,             // M1,M2,M1 -> 保持 (3个250ms单元)
    M2, M3, L6,             // M2,M3,L6 -> 保持 (3个250ms单元)
    M1, M1, L6, L6, M1, M1, L6, L6, // M1,L6,M1,L6 -> 拆分 (4个音符 -> 8个125ms单元)
    M1, M3, M2,             // M1,M3,M2 -> 保持 (3个250ms单元)
    L7, L7, S, S, S, S,     // L7,L7,S,S -> 延长S (6个250ms单元)

    // "他们说要带着光驯服每一头怪兽" (原: L6,M1,M6,M6,M6,M5,M6,M6,M5,M6,M5,M6,M5,M3,M3,M3,S,S)
    L6, M1, M6, M6,         // L6,M1,M6,M6 -> 保持 (4个250ms单元)
    M6, M5, M6, M6,         // M6,M5,M6,M6 -> 保持 (4个250ms单元)
    M5, M6, M5, M6,         // M5,M6,M5,M6 -> 保持 (4个250ms单元)
    M5, M3, M3, M3,         // M5,M3,M3,M3 -> 保持 (4个250ms单元)
    S, S, S, S,             // S,S -> 延长S (4个250ms单元)

    // "他们说要缝好你的伤没人爱小丑" (原: L6,M1,M6,M6,M6,M5,M6,M5,M7,M7,M7,M6,M7,M7,M6,M3,M3,S,S)
    L6, M1, M6, M6,         // L6,M1,M6,M6 -> 保持 (4个250ms单元)
    M6, M5, M6, M5,         // M6,M5,M6,M5 -> 保持 (4个250ms单元)
    M7, M7, M7, M6,         // M7,M7,M7,M6 -> 保持 (4个250ms单元)
    M7, M7, M6, M3,         // M7,M7,M6,M3 -> 保持 (4个250ms单元)
    M3, S, S, S,            // S,S -> 延长S (4个250ms单元)

    // "为何孤独不可光荣" (原: M3,M5,M3,M2,M3,M2,M3,M2,S)
    M3, M5, M3, M2,         // M3,M5,M3,M2 -> 保持 (4个250ms单元)
    M3, M2, M3, M2,         // M3,M2,M3,M2 -> 保持 (4个250ms单元)
    S, S,                   // S -> 延长S (2个250ms单元)

    // "人只有不完美值得歌颂" (原: M3,M5,M3,M5,M3,M2,M3,M2,M3,M2,S)
    M3, M5, M3, M5,         // M3,M5,M3,M5 -> 保持 (4个250ms单元)
    M3, M2, M3, M2,         // M3,M2,M3,M2 -> 保持 (4个250ms单元)
    M3, M2, S,              // M3,M2,S -> 保持 (3个250ms单元)

    // "谁说污泥满身的不算英雄" (原: M1,M2,M3,L6,M1,M3,M2,M3,M2,M1,M1,L6,L6,S,S)
    M1, M2, M3, L6,         // M1,M2,M3,L6 -> 保持 (4个250ms单元)
    M1, M3, M2, M3,         // M1,M3,M2,M3 -> 保持 (4个250ms单元)
    M2, M1, M1, L6,         // M2,M1,M1,L6 -> 保持 (4个250ms单元)
    L6, S, S,               // L6,S,S -> 保持 (3个250ms单元)

    // "爱你孤身走暗巷" (原: M6,M7,H1,H2,M7,H1,H1,S)
    M6, M7, H1, H2,         // M6,M7,H1,H2 -> 保持 (4个250ms单元)
    M7, H1, H1, S,          // M7,H1,H1,S -> 保持 (4个250ms单元)

    // "爱你不跪的模样" (原: H1,M7,H1,H2,M7,H1,H1,S)
    H1, M7, H1, H2,         // H1,M7,H1,H2 -> 保持 (4个250ms单元)
    M7, H1, H1, S,          // M7,H1,H1,S -> 保持 (4个250ms单元)

    // "爱你对峙过绝望不肯哭一场" (原: H1,H2,H3,H2,H3,H2,H3,H3,H2,H3,H5,H3,S)
    H1, H2, H3, H2,         // H1,H2,H3,H2 -> 保持 (4个250ms单元)
    H3, H2, H3, H3,         // H3,H2,H3,H3 -> 保持 (4个250ms单元)
    H2, H3, H5, H3,         // H2,H3,H5,H3 -> 保持 (4个250ms单元)
    S,                      // S -> 保持 (1个250ms单元)

    // "爱你破烂的衣裳却敢堵命运的枪" (原: M6,M7,H1,H2,M7,H1,H1,H1,M7,H1,H2,M7,H1,H1,S)
    M6, M7, H1, H2,         // M6,M7,H1,H2 -> 保持 (4个250ms单元)
    M7, H1, H1, H1,         // M7,H1,H1,H1 -> 保持 (4个250ms单元)
    M7, H1, H2, M7,         // M7,H1,H2,M7 -> 保持 (4个250ms单元)
    H1, H1, S,              // H1,H1,S -> 保持 (3个250ms单元)

    // "爱你和我那么像缺口一样" (原: H1,H2,H3,H2,H3,H2,H3,H3,H2,H3,H5,H3,S)
    H1, H2, H3, H2,         // H1,H2,H3,H2 -> 保持 (4个250ms单元)
    H3, H2, H3, H3,         // H3,H2,H3,H3 -> 保持 (4个250ms单元)
    H2, H3, H5, H3,         // H2,H3,H5,H3 -> 保持 (4个250ms单元)
    S,                      // S -> 保持 (1个250ms单元)

    // "去吗" (原: H5,H3)
    H5, H3,                 // H5,H3 -> 保持 (2个250ms单元)

    // "配吗" (原: H5,H3,S)
    H5, H3, S,              // H5,H3,S -> 保持 (3个250ms单元)

    // "这褴褛的披风" (原: H5,H3,H5,H6,H3,H5,S)
    H5, H3, H5, H6,         // H5,H3,H5,H6 -> 保持 (4个250ms单元)
    H3, H5, S,              // H3,H5,S -> 保持 (3个250ms单元)

    // "战吗" (原: H5,H3)
    H5, H3,                 // H5,H3 -> 保持 (2个250ms单元)

    // "战啊" (原: H5,H3,S)
    H5, H3, S,              // H5,H3,S -> 保持 (3个250ms单元)

    // "以最卑微的梦致那黑夜中的呜咽与怒吼" (原: H5,H3,H5,H6,H3,H5,H5,H5,H3,H2,H2,H2,H1,H3,H3,H2,H2,H2,H1,H1,M6,M6,S,S)
    H5, H3, H5, H6,         // H5,H3,H5,H6 -> 保持 (4个250ms单元)
    H3, H5, H5, H5,         // H3,H5,H5,H5 -> 保持 (4个250ms单元)
    H3, H2, H2, H2,         // H3,H2,H2,H2 -> 保持 (4个250ms单元)
    H1, H3, H3, H2,         // H1,H3,H3,H2 -> 保持 (4个250ms单元)
    H2, H2, H1, H1,         // H2,H2,H1,H1 -> 保持 (4个250ms单元)
    M6, M6, S, S,           // M6,M6,S,S -> 保持 (4个250ms单元)

    // "谁说站在光里才算英雄" (原: H5,H5,H3,H2,H2,H2,H1,H3,H3,H2,H2,H2,H1,H1,M6,M6,S,S)
    H5, H5, H3, H2,         // H5,H5,H3,H2 -> 保持 (4个250ms单元)
    H2, H2, H1, H3,         // H2,H2,H1,H3 -> 保持 (4个250ms单元)
    H3, H2, H2, H2,         // H3,H2,H2,H2 -> 保持 (4个250ms单元)
    H1, H1, M6, M6,         // H1,H1,M6,M6 -> 保持 (4个250ms单元)
    S, S,                   // S,S -> 保持 (2个250ms单元)
};

void Gimbal_Warning_Init(void)
{
  Buzzer_Init();
  LED_Init();LED_R_Off();LED_G_Off();LED_B_Off();
}

//写在主函数里，伪任务
void Gimbal_Warning_Remote(void)
{
  PROCESSOR(500);
  
  LED_R_Toggle();
}

void Gimbal_Warning_Can(void)
{
}

void Gimbal_Warning_BMI088(void);
void Gimbal_Warning_IST8310(void);

void Gimbal_Warning_Music(void)
{
  PROCESSOR(500);
	
	static int music_index = 0;
	music_index++;
  
	Buzzer_SetFreq(music[music_index]);
  
}

void Gimbal_Warning_Tone(uint8_t frequency,uint8_t delay_ms)
{
  PROCESSOR(delay_ms);
  
	Buzzer_SetFreq(frequency);
}


