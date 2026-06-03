/**
 * @brief  TB6612FNG 双电机驱动底层实现 (立创开发板风格)
 * * @note   硬件资源分配:
 * - 电机 A (左轮): 
 * - 方向控制: AIN1->PC1, AIN2->PC2
 * - 速度控制: PWMA->PB5 (TIM3_CH2)
 * - 电机 B (右轮): 
 * - 方向控制: BIN1->PC3, BIN2->PC4
 * - 速度控制: PWMB->PB4 (TIM3_CH1)
 * * @usage  使用说明:
 * 1. 调用 TB6612_Init(pre, per) 初始化硬件，频率建议设置在 10kHz-20kHz 避开人耳听域。
 * 计算公式: Fpwm = 168MHz / (pre * per)。
 * 2. 使用 AO_Control(dir, speed) 控制 A 电机：dir(1-正转/0-反转)，speed(0 ~ per-1)。
 * 3. 使用 BO_Control(dir, speed) 控制 B 电机：逻辑同上。
 * 4.speed (速度) 该值会被写入定时器的比较寄存器（CCR）占空比越高，电机转速越快
 * * @param  pre: 定时器预分频值 (Prescaler)
 * @param  per: 定时器自动重装载值 (Period/PWM分辨率)
 */

#include "bsp_motor.h"
#include "board.h"

void TB6612_Init(uint16_t pre,uint16_t per)
{
	GPIO_InitTypeDef 			GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	
 
	RCC_APB1PeriphClockCmd(RCC_MOTOR_TIMER,ENABLE);
 	RCC_AHB1PeriphClockCmd(RCC_PWMA | RCC_PWMB, ENABLE);   	  
	   
 
   // 2. PWMA 引脚配置 (TIM3_CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_PWMA; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(PORT_PWMA, &GPIO_InitStructure);
    GPIO_PinAFConfig(PORT_PWMA, GPIO_PWMA_SOURCE, AF_PWMA); 

    // 3. PWMB 引脚配置 (TIM3_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_PWMB; 
    GPIO_Init(PORT_PWMB, &GPIO_InitStructure);
    GPIO_PinAFConfig(PORT_PWMB, GPIO_PWMB_SOURCE, AF_PWMB); 

    // 4. 定时器基础配置
    TIM_TimeBaseStructure.TIM_Period = per - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = pre - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(BSP_MOTOR_TIMER, &TIM_TimeBaseStructure); 
    
    // 5. PWM 模式配置 (通道1和通道2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
    
    TIM_OC1Init(BSP_MOTOR_TIMER, &TIM_OCInitStructure); // PWMB
    TIM_OC2Init(BSP_MOTOR_TIMER, &TIM_OCInitStructure); // PWMA
 
    TIM_OC1PreloadConfig(BSP_MOTOR_TIMER, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(BSP_MOTOR_TIMER, TIM_OCPreload_Enable);  
 
    TIM_Cmd(BSP_MOTOR_TIMER, ENABLE);  
    
    // 6. 初始化方向控制引脚
    AIN_GPIO_INIT();
    BIN_GPIO_INIT();
}

void AIN_GPIO_INIT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AIN1 | RCC_AIN2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_AIN1 | GPIO_AIN2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(PORT_AIN1, &GPIO_InitStructure);
}

void BIN_GPIO_INIT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_BIN1 | RCC_BIN2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_BIN1 | GPIO_BIN2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(PORT_BIN1, &GPIO_InitStructure);
}

// 
// dir 旋转方向：1正转0反转 
// speed 旋转速度：范围：0~per-1
void AO_Control(uint8_t dir, uint32_t speed)
{
    if( dir == 1 ) { AIN1_OUT(0); AIN2_OUT(1); }
    else           { AIN1_OUT(1); AIN2_OUT(0); }   
    TIM_SetCompare2(BSP_MOTOR_TIMER, speed);    
}

void BO_Control(uint8_t dir, uint32_t speed)
{
    if( dir == 1 ) { BIN1_OUT(0); BIN2_OUT(1); }
    else           { BIN1_OUT(1); BIN2_OUT(0); }   
    TIM_SetCompare1(BSP_MOTOR_TIMER, speed);    
}
