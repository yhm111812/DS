#include "bsp_led.h"

/**
 * @brief 初始化天空星板载用户 LED (PB2)
 */
void LED_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    LED_OFF(); // 初始状态灭灯
}