#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "bsp_uart.h"
#include "bsp_encoder.h"
#include "bsp_timer.h"
#include "bsp_led.h"    // 👈 引入剥离出的 LED 驱动层
#include "app_motor.h"
#include "app_vision.h" 
#include "app_task.h"   // 👈 引入剥离出的 赛题逻辑层

// 引入交接角度环与速度内环控制结果的全局变量
extern float g_base_speed;   // 速度基准（主循环下发）
extern float g_turn_offset;  // 角度中环输出的差速调节分量
extern float g_yaw;          // 陀螺仪高频刷新的当前绝对角度
extern int32_t g_odom_distance_count; // 引入底层高精度里程计

// 纯双环测试目标航向（锁定直线方向）
float g_main_target_yaw = 0.0f; 

// 引入角度中环控制函数
extern float Motor_Yaw_Control_Loop(float target_yaw, float actual_yaw);

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    board_init();
    uart1_init(115200U); 
    uart2_init(115200U); 
    uart3_init(115200U); 
    
    App_Motor_Init();
    Encoder_Init_All();
    App_Vision_Init();   
    LED_Init();       
    App_Task_Init();  

    g_base_speed = 0.0f; 
    Timer_PID_Init(); // 启动 10ms 中断

    delay_ms(2000); 
    
    printf("Waiting for Vision Module...\r\n");
    while (g_vision_info.Is_Valid == 0) { delay_ms(50); }
    printf("Vision Online! Go!\r\n");

    g_main_target_yaw = g_yaw; 
    g_odom_distance_count = 0; 

    while (1)
    {
        uint32_t current_time_ms = get_tick(); 
        
        // --- 线程1：安全调参/标准数据链打印线程（移出中断，100ms安全发送） ---
        static uint32_t last_uart_time = 0;
        if (current_time_ms - last_uart_time >= 100) 
        {
            last_uart_time = current_time_ms;
            // 👈 安全的标准调试数据链打印
            printf("%.1f,%d,%.1f\r\n", g_yaw, g_odom_distance_count, g_base_speed);
        }

        // --- 🚨 线程2：原先的角度环调用被彻底删除！完全交由 10ms 中断闭环，拒绝打架 🚨 ---

        // --- 线程3：赛题四大任务统筹（20ms执行一次） ---
        static uint32_t last_stage_time = 0;
        if (current_time_ms - last_stage_time >= 20)
        {
            last_stage_time = current_time_ms;
            App_Task_Run(current_time_ms); 
        }
    }
}
