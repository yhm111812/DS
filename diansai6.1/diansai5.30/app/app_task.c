#include "app_task.h"
#include "app_motor.h"
#include "app_vision.h"
#include "bsp_led.h" 
#include <math.h>
#include <stdlib.h>

// --- 引入底层控制全局变量 ---
extern float g_base_speed;            
extern float g_yaw;                   
extern int32_t g_odom_distance_count; 
extern float g_main_target_yaw;  
extern uint8_t g_vision_enable;
extern float g_turn_offset;

// --- 外部位置环控制算子声明 ---
extern float Motor_Position_Control_Loop(float target_pos, float actual_pos);

// --- 核心里程标定宏 ---
#define CM_TO_PULSE(cm)  ((int32_t)((cm) * 66.6f)) 

// 全局任务模式定义
RaceMode_e g_current_mode = MODE_WAIT; 

// 内部独立子任务函数声明
static void Run_Task_1(uint32_t current_time_ms);
static void Run_Task_2(uint32_t current_time_ms);
static void Run_Task_3(uint32_t current_time_ms); 
static void Run_Task_4(uint32_t current_time_ms); 

void App_Task_Init(void)
{
    // 比赛/测试调试时，修改此枚举切换赛题任务
    g_current_mode = MODE_TASK_2; 
}

void App_Task_Run(uint32_t current_time_ms)
{
    switch (g_current_mode)
    {
        case MODE_WAIT:   g_base_speed = 0.0f; g_vision_enable = 0; break;
        case MODE_TASK_1: Run_Task_1(current_time_ms); break;
        case MODE_TASK_2: Run_Task_2(current_time_ms); break;
        case MODE_TASK_3: Run_Task_3(current_time_ms); break;
        case MODE_TASK_4: Run_Task_4(current_time_ms); break;
    }
}

// =========================================================================
// 🚀 任务1：A点到B点停车（0打架、纯陀螺仪刚度）
// =========================================================================
static void Run_Task_1(uint32_t current_time_ms)
{
    static uint8_t step = 0;
    static uint32_t led_timer = 0;

    switch (step)
    {
        case 0: // A -> B 直线盲跑
            g_vision_enable = 0; // 强制关闭视觉
            g_main_target_yaw = 0.0f; 
            g_base_speed = Motor_Position_Control_Loop(CM_TO_PULSE(100.0f), g_odom_distance_count);
            
            if (abs(g_odom_distance_count - CM_TO_PULSE(100.0f)) < CM_TO_PULSE(2.0f)) 
            {
                step = 1; led_timer = current_time_ms;
            }
            break;

        case 1: // B点停车光提示
            g_base_speed = 0.0f; LED_ON();
            if (current_time_ms - led_timer > 500) 
            {
                LED_OFF(); step = 2; 
            }
            break;

        case 2: g_base_speed = 0.0f; break; 
    }
}

// =========================================================================
// 🚀 任务2：A -> B -> C -> D -> A 正常顺时针循环跑圈
// =========================================================================
static void Run_Task_2(uint32_t current_time_ms)
{
    static uint8_t step = 0;
    static uint32_t led_timer = 0;
    static uint32_t stable_timer = 0; 
    
    float current_err = 0.0f;

    switch (step)
    {
        case 0: // A -> B (盲跑 100cm)
            g_vision_enable = 0; 
            g_main_target_yaw = 0.0f;
            g_base_speed = Motor_Position_Control_Loop(CM_TO_PULSE(100.0f), g_odom_distance_count);
            if (abs(g_odom_distance_count - CM_TO_PULSE(100.0f)) < CM_TO_PULSE(2.0f)) {
                step = 1; led_timer = current_time_ms;
            }
            break;

        case 1: // B点亮灯保持，重置里程，准备入弯
            g_base_speed = 0.0f; LED_ON();
            if (current_time_ms - led_timer > 500) {
                LED_OFF(); g_odom_distance_count = 0; step++;
            }
            break;

        case 2: // B -> C (弯道巡线过弯)
            g_vision_enable = 1; // 激活视觉外环
            g_main_target_yaw = g_yaw; // 目标角度跟随当前，防止与视觉打架内耗
            g_base_speed = 6.0f; 
            
            // 里程达标且进入反向大范围
            if (g_odom_distance_count >= CM_TO_PULSE(110.0f) && (g_yaw >= 140.0f || g_yaw <= -140.0f)) {
                step = 3; led_timer = current_time_ms;
            }
            break;

        case 3: // C点出弯：强行原地硬拧 + 150ms 机械泄能平行锁
            g_vision_enable = 0;        // 1. 强切瞬间将视觉彻底断开，抹除所有残余图像波动！
            g_base_speed = 4.0f;        // 2. 给予 4.0f 的基础滑行速度，满足左右轮向前滚的底线
            
            // 🚨【全场完赛核心修正 1：目标航向精准对齐负世界】🚨
            // 因为小车向右拐弯是 0 到负 180 度，所以到达 C 点时，车头实际是在 -171 度左右！
            // 为了让代数做减法时符号完全对齐，我们将盲冲大直线的目标方向精准写成 -180.0f！
            g_main_target_yaw = -180.0f; 

            // 🚨【全场完赛核心修正 2：用标准的目标减实际，触发正反馈】🚨
            // 此时计算：目标 (-180.0) - 实际 (-171.0) = -9.0f！
            // 负数的角度误差传导给底层 (g_base_speed - offset) 之后，负负得正！
            // 右轮 (4.0 - (-9.0)) 会自
            // 只要当前的物理误差绝对值大于 2.5 度（还没拧正），就一直刷新 stable_timer 重新计时
            if (fabs(current_err) > 2.5f) 
            {
                stable_timer = current_time_ms; 
            }
            
            // 唯有当车身在级联的正确反馈下，真正向右扭正进圈了，且连续稳定保持满了 150 毫秒！
            // 左右轮刚才由于激烈纠偏产生的转速差在地上完全摩擦平复，车轮机械上绝对平行了，才放行！
            if (current_time_ms - stable_timer >= 150) 
            {
                g_turn_offset = 0.0f; // 刷白动轰大油门前推，左轮 (4.0 + (-9.0)) 会自动慢下来带路！
            // 终于可以完美触发你预期的“左轮慢一些、右轮快一些，通过差速向右扭正车头”！
            current_err = g_main_target_yaw - g_yaw; 
            
            // 电赛级标准防暴走角度过零跳变保护
            while(current_err > 180.0f)  current_err -= 360.0f;
            while(current_err < -180.0f) current_err += 360.0f;
                g_odom_distance_count = 0; 
                step = 4; // 轮子彻底正了，正式放行盲冲大直线！
            }
            break;

        case 4: // C -> D (反向大直道盲跑 100cm)
            g_vision_enable = 0; 
            g_main_target_yaw = -180.0f; 
            g_base_speed = Motor_Position_Control_Loop(CM_TO_PULSE(100.0f), g_odom_distance_count);
            if (abs(g_odom_distance_count - CM_TO_PULSE(100.0f)) < CM_TO_PULSE(2.0f)) {
                step = 5; led_timer = current_time_ms;
            }
            break;

        case 5: // D点 亮灯重置
            g_base_speed = 0.0f; LED_ON();
            if (current_time_ms - led_timer > 500) {
                LED_OFF(); g_odom_distance_count = 0; step++;
            }
            break;

        case 6: // D -> A (顺向弯道巡线返回)
            g_vision_enable = 1; 
            g_main_target_yaw = g_yaw; // 目标跟随实际，拒绝拔河
            g_base_speed = 6.0f; 
            
            if (g_odom_distance_count >= CM_TO_PULSE(110.0f) && (fabs(g_yaw) <= 10.0f)) {
                step = 7; led_timer = current_time_ms;
            }
            break;

        case 7: // A点 终点动作
            g_vision_enable = 0; 
            g_base_speed = 0.0f; LED_ON();
            if (current_time_ms - led_timer > 500) {
                LED_OFF(); 
                g_main_target_yaw = 0.0f; 
                step = 8;
            }
            break;

        case 8: g_base_speed = 0.0f; g_vision_enable = 0; break;
    }
}

// =========================================================================
// 🚀 任务3：A -> C -> B -> D -> A (对角线交叉斜穿，包含逆向过弯)
// =========================================================================
static void Run_Task_3(uint32_t current_time_ms)
{
    static uint8_t step = 0;
    static uint32_t led_timer = 0;
    static uint32_t stable_timer = 0; // 新增任务3专属硬等时间锁
    
    float current_err = 0.0f;

    switch (step)
    {
        case 0: // A -> C 交叉大对角线盲跑
            g_vision_enable = 0; 
            g_main_target_yaw = 38.6f; 
            g_base_speed = Motor_Position_Control_Loop(CM_TO_PULSE(128.0f), g_odom_distance_count);
            if (abs(g_odom_distance_count - CM_TO_PULSE(128.0f)) < CM_TO_PULSE(2.0f)) {
                step = 1; led_timer = current_time_ms;
            }
            break;

        case 1: // C点等灯指示
            g_base_speed = 0.0f; LED_ON();
            g_vision_enable = 0;
            if (current_time_ms - led_timer > 500) {
                LED_OFF(); g_odom_distance_count = 0; step++;
            }
            break;

        case 2: // C -> B 逆向巡线弯道（核心修正处）
            g_vision_enable = 1;       // 🚨【修复】：必须赋予底层看线的权利，否则直接变瞎子冲场！
            g_main_target_yaw = g_yaw; // 🚨【修复】：必须让航向跟随当前，彻底消灭双环内耗打摆子！
            g_base_speed = 5.5f; 
            if (g_odom_distance_count >= CM_TO_PULSE(125.6f)) {
                step = 3; led_timer = current_time_ms;
            }
            break;

        case 3: // 🚨【重大新增】：B点出弯车轮平复锁，干掉逆向弯强切引起的所有相反偏转！
            g_vision_enable = 0;       // 视觉斩断
            g_base_speed = 0.0f;       // 原地泄能
            
            // 逆向跑圈出弯时，车头物理上朝向后方直道。
            // 依据顺逆时针的解算原理，此时锁定的对齐大方向应当为 -180.0f！
            g_main_target_yaw = -180.0f; 

            // 计算与 -180.0f 的真实控制误差
            current_err = -180.0f - g_yaw; 
            while(current_err > 180.0f)  current_err -= 360.0f;
            while(current_err < -180.0f) current_err += 360.0f;

            if (fabs(current_err) > 2.5f) {
                stable_timer = current_time_ms; 
            }
            
            // 在原地憋足 150 毫秒，强行让轮子恢复绝对平行状态！
            if (current_time_ms - stable_timer >= 150) {
                g_turn_offset = 0.0f; 
                g_odom_distance_count = 0; 
                step = 4; // 轮子扭正了，放行！
            }
            break;

        case 4: // B -> D 交叉对角线斜穿盲跑
            g_vision_enable = 0; 
            g_main_target_yaw = -38.6f; 
            g_base_speed = Motor_Position_Control_Loop(CM_TO_PULSE(128.0f), g_odom_distance_count);
            if (abs(g_odom_distance_count - CM_TO_PULSE(128.0f)) < CM_TO_PULSE(2.0f)) {
                step = 5; led_timer = current_time_ms;
            }
            break;

        case 5: // D点等灯指示
            g_base_speed = 0.0f; LED_ON();
            g_vision_enable = 0;
            if (current_time_ms - led_timer > 500) {
                LED_OFF(); g_odom_distance_count = 0; step++;
            }
            break;

        case 6: // D -> A 顺向弯道巡线返回
            g_vision_enable = 1;       // 🚨【修复】：开视觉
            g_main_target_yaw = g_yaw; // 🚨【修复】：防打架
            g_base_speed = 5.5f; 
            if (g_odom_distance_count >= CM_TO_PULSE(125.6f)) {
                step = 7; led_timer = current_time_ms;
            }
            break;

        case 7: // 回到A点，单圈结束
            g_vision_enable = 0;
            g_base_speed = 0.0f; LED_ON();
            if (current_time_ms - led_timer > 500) {
                LED_OFF(); step = 8;
            }
            break;

        case 8: g_base_speed = 0.0f; g_vision_enable = 0; break;
    }
}

// =========================================================================
// 🚀 任务4：高刚度、大油门循环跑 4 圈任务3（无任何时序割裂）
// =========================================================================
static void Run_Task_4(uint32_t current_time_ms)
{
    static uint8_t step = 0;
    static uint32_t led_timer = 0;
    static uint32_t stable_timer = 0; // 任务4专属硬等时间锁
    static uint8_t lap_count = 0; 
    
    float current_err = 0.0f;

    switch (step)
    {
        case 0: // A -> C 交叉斜穿
            g_vision_enable = 0; 
            g_main_target_yaw = 38.6f; 
            g_base_speed = Motor_Position_Control_Loop(CM_TO_PULSE(128.0f), g_odom_distance_count);
            if (abs(g_odom_distance_count - CM_TO_PULSE(128.0f)) < CM_TO_PULSE(2.0f)) {
                step = 1; led_timer = current_time_ms;
            }
            break;

        case 1: 
            g_base_speed = 0.0f; LED_ON(); g_vision_enable = 0;
            if (current_time_ms - led_timer > 500) {
                LED_OFF(); g_odom_distance_count = 0; step++;
            }
            break;

        case 2: // C -> B 逆向巡线
            g_vision_enable = 1;       // 🚨【修复】：激活视觉
            g_main_target_yaw = g_yaw; // 🚨【修复】：防互殴
            g_base_speed = 5.5f; 
            if (g_odom_distance_count >= CM_TO_PULSE(125.6f)) {
                step = 3; led_timer = current_time_ms;
            }
            break;

        case 3: // 🚨【重大新增】：任务4过弯强切物理平复锁！
            g_vision_enable = 0; 
            g_base_speed = 0.0f; 
            g_main_target_yaw = -180.0f; 

            current_err = -180.0f - g_yaw; 
            while(current_err > 180.0f)  current_err -= 360.0f;
            while(current_err < -180.0f) current_err += 360.0f;

            if (fabs(current_err) > 2.5f) {
                stable_timer = current_time_ms; 
            }
            if (current_time_ms - stable_timer >= 150) {
                g_turn_offset = 0.0f; 
                g_odom_distance_count = 0; 
                step = 4; // 轮子平行，安全起跑！
            }
            break;

        case 4: // B -> D 交叉斜穿盲跑
            g_vision_enable = 0; 
            g_main_target_yaw = -38.6f; 
            g_base_speed = Motor_Position_Control_Loop(CM_TO_PULSE(128.0f), g_odom_distance_count);
            if (abs(g_odom_distance_count - CM_TO_PULSE(128.0f)) < CM_TO_PULSE(2.0f)) {
                step = 5; led_timer = current_time_ms;
            }
            break;

        case 5: 
            g_base_speed = 0.0f; LED_ON(); g_vision_enable = 0;
            if (current_time_ms - led_timer > 500) {
                LED_OFF(); g_odom_distance_count = 0; step++;
            }
            break;

        case 6: // D -> A 顺向巡线
            g_vision_enable = 1;       // 🚨【修复】：激活视觉
            g_main_target_yaw = g_yaw; // 🚨【修复】：防打架
            g_base_speed = 5.5f; 
            if (g_odom_distance_count >= CM_TO_PULSE(125.6f)) {
                step = 7; led_timer = current_time_ms;
            }
            break;

        case 7: // 回到A点，处理圈数累加
            g_vision_enable = 0;
            g_base_speed = 0.0f; LED_ON();
            if (current_time_ms - led_timer > 500) 
            {
                LED_OFF(); 
                lap_count++; 
                
                if (lap_count < 4) 
                {
                    step = 0;                  // 踢回阶段0，开始下一圈
                    g_odom_distance_count = 0; 
                } 
                else 
                {
                    step = 8; // 跑满4圈去终点
                }
            }
            break;

        case 8: g_base_speed = 0.0f; g_vision_enable = 0; break;
    }
}
