#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__


#include "stm32f4xx.h"
#include "board.h"

#define RCC_AIN1                RCC_AHB1Periph_GPIOC
#define PORT_AIN1               GPIOC
#define GPIO_AIN1               GPIO_Pin_1

#define RCC_AIN2                RCC_AHB1Periph_GPIOC
#define PORT_AIN2               GPIOC
#define GPIO_AIN2               GPIO_Pin_2

#define RCC_BIN1                RCC_AHB1Periph_GPIOC
#define PORT_BIN1               GPIOC
#define GPIO_BIN1               GPIO_Pin_3

#define RCC_BIN2                RCC_AHB1Periph_GPIOC
#define PORT_BIN2               GPIOC
#define GPIO_BIN2               GPIO_Pin_4

#define RCC_PWMA                RCC_AHB1Periph_GPIOB
#define PORT_PWMA               GPIOB
#define GPIO_PWMA               GPIO_Pin_5
#define GPIO_PWMA_SOURCE		GPIO_PinSource5
#define AF_PWMA                 GPIO_AF_TIM3

#define RCC_PWMB                RCC_AHB1Periph_GPIOB
#define PORT_PWMB               GPIOB
#define GPIO_PWMB               GPIO_Pin_4
#define GPIO_PWMB_SOURCE        GPIO_PinSource4
#define AF_PWMB                 GPIO_AF_TIM3

#define RCC_MOTOR_TIMER         RCC_APB1Periph_TIM3
#define BSP_MOTOR_TIMER         TIM3

#define AIN1_OUT(X)  GPIO_WriteBit(PORT_AIN1, GPIO_AIN1, X?Bit_SET:Bit_RESET)
#define AIN2_OUT(X)  GPIO_WriteBit(PORT_AIN2, GPIO_AIN2, X?Bit_SET:Bit_RESET)
#define BIN1_OUT(X)  GPIO_WriteBit(PORT_BIN1, GPIO_BIN1, (X?Bit_SET:Bit_RESET))
#define BIN2_OUT(X)  GPIO_WriteBit(PORT_BIN2, GPIO_BIN2, (X?Bit_SET:Bit_RESET))

void AIN_GPIO_INIT(void);
void BIN_GPIO_INIT(void);
void TB6612_Init(uint16_t pre,uint16_t per);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

#endif
