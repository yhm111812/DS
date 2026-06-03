#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include "stm32f4xx.h"
#include "board.h"

// --- 左轮编码器 (TIM2: PA0, PA1) ---
#define RCC_ENCODER_L_GPIO    RCC_AHB1Periph_GPIOA
#define RCC_ENCODER_L_TIMER   RCC_APB1Periph_TIM2
#define PORT_ENCODER_L        GPIOA
#define GPIO_ENC_L_A          GPIO_Pin_0
#define GPIO_ENC_L_B          GPIO_Pin_1
#define GPIO_ENC_L_A_SOURCE   GPIO_PinSource0
#define GPIO_ENC_L_B_SOURCE   GPIO_PinSource1
#define AF_ENCODER_L          GPIO_AF_TIM2
#define ENCODER_L_TIMER       TIM2

// --- 右轮编码器 (TIM4: PB6, PB7) ---
#define RCC_ENCODER_R_GPIO    RCC_AHB1Periph_GPIOB
#define RCC_ENCODER_R_TIMER   RCC_APB1Periph_TIM4
#define PORT_ENCODER_R        GPIOB
#define GPIO_ENC_R_A          GPIO_Pin_6
#define GPIO_ENC_R_B          GPIO_Pin_7
#define GPIO_ENC_R_A_SOURCE   GPIO_PinSource6
#define GPIO_ENC_R_B_SOURCE   GPIO_PinSource7
#define AF_ENCODER_R          GPIO_AF_TIM4
#define ENCODER_R_TIMER       TIM4

// 函数声明
void Encoder_Init_All(void);          // 初始化所有编码器
int16_t Encoder_Get_Speed_Left(void);  // 获取左轮速度（脉冲数）
int16_t Encoder_Get_Speed_Right(void); // 获取右轮速度（脉冲数）

#endif
