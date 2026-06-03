#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "board.h"

// 纯光报警宏定义 (操作 PB2)
#define LED_ON()  GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define LED_OFF() GPIO_ResetBits(GPIOB, GPIO_Pin_2)

void LED_Init(void);

#endif
