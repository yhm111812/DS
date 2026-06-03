#include "bsp_timer.h"
#include "app_motor.h" // 声明了你的 PID 控制函数

/**
 * @brief  初始化定时器1，用于20ms一次的PID控制周期
 * @note   TIM1在APB2总线上，时钟频率168MHz
 */
void Timer_PID_Init(void)
{
    // 1. 开启时钟 (注意：F407的TIM1在APB2上)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    // 2. 时基单元配置
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    /* 计算逻辑：
       时钟频率 = 168MHz
       PSC = 1680-1  => 计数频率 = 168MHz / 1680 = 100KHz (10us计一次数)
       ARR = 1000-1  => 计1000个数 = 1000 * 10us = 10,000us = 10ms
    */
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;       // 自动重装值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1680 - 1;    // 预分频值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;   // 高级定时器特有
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    // 3. 清除更新标志位，防止初始化完立即进入中断
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    
    // 4. 开启定时器更新中断
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    
     // 5. NVIC配置 (注意：TIM1的更新中断通道名称与F1不同)
    // 如果你已经配置过优先级分组(NVIC_PriorityGroupConfig)，这里直接填优先级

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
    // ⚠️ 极其关键的修改：将抢占优先级从 0 降为 2
    // 务必保证你的 bsp_uart.c 里面，陀螺仪串口的优先级是 0 或 1！
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    
    // 6. 启动定时器
    TIM_Cmd(TIM1, ENABLE);
}

/**
 * @brief  定时器1的中断服务函数
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    // 检查更新中断标志位
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        /* 【内环稳速心跳】
           这里直接调用在 app_motor.c 中重构好的独立内环控制函数。
           该函数内部包含了：
           1. 准时读取两轮编码器，读完底层自动执行计数清零（即刻清空反馈）。
           2. 异步抓取由 main 循环计算并下发的 g_base_speed 和 g_turn_offset 组合成各自的目标速度。
           3. 跑高级增量式速度 PID（带积分饱和抑制、输出物理限幅和输出死区消抖）。
           4. 物理输出 PWM 写入电机硬件。
        */
        Motor_Speed_Control_PID();
        
        // 清除标志位
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
