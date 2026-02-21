#ifndef GIMBAL_WARNING_H
#define GIMBAL_WARNING_H
#include <stdint.h>

void Gimbal_Warning_Init(void);
void Gimbal_Warning_Remote(void);
void Gimbal_Warning_Can(void);
void Gimbal_Warning_BMI088(void);
void Gimbal_Warning_IST8310(void);

void Gimbal_Warning_Music(void);
void Gimbal_Warning_Tone(uint8_t frequency,uint8_t delay_ms);

#endif // GIMBAL_WARNING_H

