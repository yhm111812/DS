#include <stdio.h>
#include <stdlib.h>
#include "app_motor.h"
#include "bsp_motor.h" // 包含之前定义的双路驱动头文件
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "board.h"
#include "math.h"
#include "app_vision.h" // 【新增】引入视觉控制头文件

// 引入在 bsp_uart.c 中由陀螺仪中断高频更新的当前真实绝对航向角
extern float g_yaw; 

// 【新增】引入 main.c 中开机死等锁定的绝对直线参考目标角度
extern float g_main_target_yaw;

/**
 * @brief  视觉丢失后，根据当前 yaw 自动选择最近的后向直线目标
 */
float Motor_Get_LostLine_Target_Yaw(float current_yaw)
{
    if (current_yaw >= 0.0f)
    {
        return 180.0f;
    }
    else
    {
        return -180.0f;
    }
}

// 级联控制核心：三环之间内部交接的全局中间变量
float g_turn_offset = 0.0f; // 角度环输出的差速调节量（内环直接提取）
float g_base_speed  = 0.0f; // 基础前进期望速度

// 新增高精度里程累加器（采用 int32_t 彻底规避 float 精度耗尽引起的飞车硬伤）
 int32_t g_odom_distance_count = 0;

// 🚨【重构核心变量】：视觉外环全勤使能开关
// 1：激活视觉最外环，允许图像偏差拉扯航向（用于所有的过弯巡线 case）
// 0：关闭视觉最外环，视觉彻底闭嘴，航向死死锁死（用于所有的盲跑、回正、等灯 case）
uint8_t g_vision_enable = 0;



/* 1. 内环速度环高级结构体*/
typedef struct {
    float Target_Speed;    // 目标速度（脉冲数/10ms）
    float Kp, Ki, Kd;      // PID三项系数
    float Error, Last_Error; 
    float Integral_Sum;    
    
    float Integral_Max;    // 积分上限幅线（抗饱和）
    float Output_Max;      // 寄存器硬件物理上限幅（950）
    float Dead_Band;       // 输出极小消抖死区
} Speed_PID_t;

/* 2. 外层位置与角度环结构体 */
typedef struct {
    float Target;          // 期望目标值
    float Kp, Ki, Kd;      // PID参数
    float Error, Last_Error, Prev_Error;
    float Output_Max;      // 最大限制输出
} Pos_Angle_PID_t;


// 积分上限400 (留出空间做差速), 物理上限950, 死区15
Speed_PID_t pid_L = {0.0f, 15.0f, 1.2f, 0.0f, 0.0f, 0.0f, 0.0f, 400.0f, 950.0f, 15.0f};
Speed_PID_t pid_R = {0.0f, 15.0f, 1.2f, 0.0f, 0.0f, 0.0f, 0.0f, 400.0f, 950.0f, 15.0f};

// 初始化角度环和位置环结构体（保持你原本的电赛级参数配置不变）
Pos_Angle_PID_t pid_Yaw = {0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 500.0f}; // 角度环：最大差速限制500
Pos_Angle_PID_t pid_Pos = {0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 35.0f};  // 位置环：单次最大偏航修正限制35度

/**
 * @brief  电机底层驱动外设初始化
 */
void App_Motor_Init(void)
{
    // 初始化底层驱动：预分频168，周期1000
    // 频率 = 168MHz / (168 * 1000) = 1kHz
    TB6612_Init(168, 1000);
}

/**
 * @brief  【最外环：位置环】PID控制函数
 */
float Motor_Position_Control_Loop(float target_pos, float actual_pos)
{
    pid_Pos.Target = target_pos;
    pid_Pos.Error = pid_Pos.Target - actual_pos;
    
    // 采用纯 P 控制，防止外环引入积分在大负载惯性下引发刹车过冲
    float pos_output = pid_Pos.Kp * pid_Pos.Error;
    
    // 位置环输出上限幅（防止单次偏差过大导致车头产生毁灭性的剧烈摆动）
    if(pos_output > pid_Pos.Output_Max)  pos_output = pid_Pos.Output_Max;
    if(pos_output < -pid_Pos.Output_Max) pos_output = -pid_Pos.Output_Max;
    
    pid_Pos.Last_Error = pid_Pos.Error;
    return pos_output; 
}

/**
 * @brief  【中间环：角度环】PID控制函数
 */
float Motor_Yaw_Control_Loop(float target_yaw, float actual_yaw)
{
    pid_Yaw.Target = target_yaw;
    pid_Yaw.Error = pid_Yaw.Target - actual_yaw;
    
    // 电赛级防摔车关键：角度过零保护（解决 ±180 度边界线跳变引发的控制暴走）
    if(pid_Yaw.Error > 180.0f)  pid_Yaw.Error -= 360.0f;
    if(pid_Yaw.Error < -180.0f) pid_Yaw.Error += 360.0f;

    // 计算角度环纯 P 调节分量（内环本身自带积分，外环纯P可做到无静差）
    g_turn_offset = pid_Yaw.Kp * pid_Yaw.Error;
    
    // 角度环输出上限幅（限制左右轮最大由于偏航产生的差速上限）
    if(g_turn_offset > pid_Yaw.Output_Max)  g_turn_offset = pid_Yaw.Output_Max;
    if(g_turn_offset < -pid_Yaw.Output_Max) g_turn_offset = -pid_Yaw.Output_Max;
    
    pid_Yaw.Last_Error = pid_Yaw.Error;
    return g_turn_offset;
}

/**
 * @brief  【内环算子：标准高级位置式速度 PID 控制算法】
 * @return 吐出带有正负号（-950 ~ 950）的物理控制量，正数代表往前推，负数代表反向电磁刹车/倒车
 */
float PID_Position_Compute(Speed_PID_t *pid, float actual_val)
{
    float final_output;
    
    // 1. 计算当前的单轮速度转转误差
    pid->Error = pid->Target_Speed - actual_val;

    // 2. 位置式核心：将当前误差直接累加进历史积分和中
    pid->Integral_Sum += pid->Error;
    
    // 3. 积分抗饱和限制必须在参与乘法计算前拦截，防止计算振荡过冲
    if (pid->Integral_Sum > pid->Integral_Max)  pid->Integral_Sum = pid->Integral_Max;
    if (pid->Integral_Sum < -pid->Integral_Max) pid->Integral_Sum = -pid->Integral_Max;
    
    // 4. 位置式标准三项计算
    final_output = (pid->Kp * pid->Error) + \
                   (pid->Ki * pid->Integral_Sum) + \
                   (pid->Kd * (pid->Error - pid->Last_Error));
    
    // 5. 【完美的双向物理限制】：限制电压 -950 ~ 950，赋予它在内环倒车和踩电磁刹车的能力
    if (final_output > pid->Output_Max)  final_output = pid->Output_Max;
    if (final_output < -pid->Output_Max) final_output = -pid->Output_Max; 

    // 6. 【输出消抖死区处理】：使用 fabs 确保负电压死区也能被完美拦截
    if (fabs(final_output) < pid->Dead_Band) {
        final_output = 0.0f;
        pid->Integral_Sum = 0.0f; // 【完美修复】消除静息状态下的积分暗中累加，防止车轮在死区蓄力抽动
    }

    // 7. 滚动更新历史记录
    pid->Last_Error = pid->Error;    
    
    return final_output;
}

/**
 * @brief  【内环：高频稳速核心心跳】
 * @note   由 TIM1 定时器中断每 10ms 强行触发一次，纯粹轻量，绝不允许加延时。
 */
void Motor_Speed_Control_PID(void)
{
    // 1. 高频读取当前左右轮编码器速度
    int16_t real_L = Encoder_Get_Speed_Left();
    int16_t real_R = Encoder_Get_Speed_Right(); 
    
    // 总里程高精度累加（位置环的地基）
    g_odom_distance_count += (int32_t)real_L;
	
// ========================================================
    // 2. 级联目标分配（20ms严格分频，拒绝与陀螺仪抢占打架）
    // ========================================================
    static uint8_t loop_div_20ms = 0;
    static uint8_t lost_line_mode = 0;   // 1：视觉丢失保护模式
    loop_div_20ms++;

    // 默认使用任务层下发的速度
    float final_base_speed = g_base_speed;
    
    if (loop_div_20ms >= 2)
    {
        loop_div_20ms = 0; 

        float yaw_compensation = 0.0f;
        float final_target_yaw = g_main_target_yaw;
  
        // 只要任务层允许视觉，才检查视觉是否在线
        if (g_vision_enable == 1) 
        {
            if (Vision_Check_Timeout() == 1) 
            {
                // 视觉正常：视觉外环输出航向补偿
                yaw_compensation = Vision_Control_Loop(g_vision_info.Error_X);
                final_target_yaw = g_main_target_yaw + yaw_compensation;

                lost_line_mode = 0;
            } 
            else 
            {
                // 视觉丢失：强制回正到后向直线方向 ±180°
                yaw_compensation = 0.0f;
                final_target_yaw = Motor_Get_LostLine_Target_Yaw(g_yaw);

                // 标记进入丢线保护模式
                lost_line_mode = 1;
            }
        }
        else 
        {
            // 状态机下令关闭视觉：纯陀螺仪锁航向
            yaw_compensation = 0.0f;
            final_target_yaw = g_main_target_yaw;

            lost_line_mode = 0;
        }
        
        // 航向中环在中断内唯一安全执行
        g_turn_offset = Motor_Yaw_Control_Loop(final_target_yaw, g_yaw);
    }

    // 丢线后降速，减少左右晃动
    if (lost_line_mode == 1 && g_vision_enable == 1)
    {
        final_base_speed = 3.5f;
    }

    // 基础前进速度与差速项叠加分配
    float target_L_temp = final_base_speed + g_turn_offset;
    float target_R_temp = final_base_speed - g_turn_offset;

    pid_L.Target_Speed = target_L_temp; 
    pid_R.Target_Speed = target_R_temp;
    
    // 位置式 Speed PID 计算输出
    float out_L = PID_Position_Compute(&pid_L, (float)real_L);
    float out_R = PID_Position_Compute(&pid_R, (float)real_R);

    // 翻译写入底层驱动硬件寄存器
    if (out_L >= 0.0f) AO_Control(1, (uint32_t)out_L);
    else               AO_Control(0, (uint32_t)(-out_L));

    if (out_R >= 0.0f) BO_Control(1, (uint32_t)out_R);
    else               BO_Control(0, (uint32_t)(-out_R));

    // 🚨【安全警告】：原先这里的 printf 已经被彻底连根拔起删除了！
    // 保证中断内执行时间低于 0.1ms，绝对不阻塞主循环和状态机时序！
}
