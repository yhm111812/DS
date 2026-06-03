#include "bsp_encoder.h"
#include "board.h"
/**
 * @brief  初始化所有编码器硬件接口
 */
void Encoder_Init_All(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 开启时钟
    RCC_AHB1PeriphClockCmd(RCC_ENCODER_L_GPIO | RCC_ENCODER_R_GPIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_ENCODER_L_TIMER | RCC_ENCODER_R_TIMER, ENABLE);

    // 2. 配置左轮引脚 (PA0, PA1)
    GPIO_InitStructure.GPIO_Pin = GPIO_ENC_L_A | GPIO_ENC_L_B;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(PORT_ENCODER_L, &GPIO_InitStructure);

    // 3. 配置右轮引脚 (PB6, PB7)
    GPIO_InitStructure.GPIO_Pin = GPIO_ENC_R_A | GPIO_ENC_R_B;
    GPIO_Init(PORT_ENCODER_R, &GPIO_InitStructure);

    // 4. 设置引脚复用映射
    GPIO_PinAFConfig(PORT_ENCODER_L, GPIO_ENC_L_A_SOURCE, AF_ENCODER_L);
    GPIO_PinAFConfig(PORT_ENCODER_L, GPIO_ENC_L_B_SOURCE, AF_ENCODER_L);
    GPIO_PinAFConfig(PORT_ENCODER_R, GPIO_ENC_R_A_SOURCE, AF_ENCODER_R);
    GPIO_PinAFConfig(PORT_ENCODER_R, GPIO_ENC_R_B_SOURCE, AF_ENCODER_R);

    // 5. 配置定时器为编码器接口模式 (TIM_EncoderMode_TI12 为 4 倍频)
    TIM_EncoderInterfaceConfig(ENCODER_L_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(ENCODER_R_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 6. 清零并启动
    TIM_SetCounter(ENCODER_L_TIMER, 0);
    TIM_SetCounter(ENCODER_R_TIMER, 0);
    TIM_Cmd(ENCODER_L_TIMER, ENABLE);
    TIM_Cmd(ENCODER_R_TIMER, ENABLE);
}

/**
 * @brief  获取左轮编码器增量并清零
 */
int16_t Encoder_Get_Speed_Left(void)
{
    int16_t count = (int16_t)TIM_GetCounter(ENCODER_L_TIMER);
    TIM_SetCounter(ENCODER_L_TIMER, 0);
    return count;
}

/**
 * @brief  获取右轮编码器增量并清零
 */
int16_t Encoder_Get_Speed_Right(void)
{
    int16_t count = (int16_t)TIM_GetCounter(ENCODER_R_TIMER);
    TIM_SetCounter(ENCODER_R_TIMER, 0);
    return -count;
}

