#ifndef LED_H
#define LED_H
#include "main.h"

void LED_Init(void);

void LED_R_On(void);
void LED_R_Off(void);
void LED_R_Toggle(void);

void LED_G_On(void);
void LED_G_Off(void);
void LED_G_Toggle(void);

void LED_B_On(void);
void LED_B_Off(void);
void LED_B_Toggle(void);

void LED_SetColor(uint8_t r, uint8_t g, uint8_t b);



#endif // LED_H

