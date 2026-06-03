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
// =========================================================
// 任务3/4：高速斜穿 + 回正直走寻线 参数
// =========================================================

// 斜穿角度
#define TASK3_YAW_A_TO_C        (-38.6f)
#define TASK3_YAW_B_TO_D        (-141.4f)

// 斜穿后寻线前的直走方向
// A -> C 到 C 点后，准备进入 C -> B 弧线，车头修正到 0°
#define TASK3_YAW_C_SEEK        (0.0f)

// B -> D 到 D 点后，准备进入 D -> A 弧线，车头修正到 -180°
#define TASK3_YAW_D_SEEK        (-180.0f)

// 速度
#define TASK3_DIAG_SPEED        (15.0f)   // 斜穿速度，按你的要求改成 15.0f
#define TASK3_ALIGN_SPEED       (3.0f)    // 回正时速度，不能太快
#define TASK3_SEEK_SPEED        (8.0f)    // 直走寻线速度，建议不要 15，否则会继续冲过线
#define TASK3_ARC_SPEED         (15.0f)    // 弯道巡线速度

// 距离
// 因为斜穿速度变成 15，不能再跑满 128cm，否则必然冲过头
#define TASK3_DIAG_FAST_CM      (100.0f)   // 高速斜穿距离
#define TASK3_DIAG_MAX_CM       (135.0f)  // 最多寻线距离，防死等

// 弯道判定
#define TASK3_ARC_MIN_CM        (85.0f)
#define TASK3_ARC_MAX_CM        (138.0f)

// yaw 判断
#define TASK3_BACK_YAW_OK()     ((g_yaw >= 150.0f) || (g_yaw <= -150.0f))
#define TASK3_FRONT_YAW_OK()    (fabs(g_yaw) <= 18.0f)




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
    g_current_mode = MODE_TASK_3; 
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
            g_base_speed = 15.0f; 
            
            // 里程达标且进入反向大范围
            if (g_odom_distance_count >= CM_TO_PULSE(100.0f) && (g_yaw >= 140.0f || g_yaw <= -140.0f)) {
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
            g_base_speed = 15.0f; 
            
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

static float Task_Get_Yaw_Error(float target_yaw, float actual_yaw)
{
    float err = target_yaw - actual_yaw;

    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    return err;
}





// =========================================================================
// 🚀 任务3：A -> C -> B -> D -> A
// 高速斜穿 + 回正直走寻线容错版
// =========================================================================
static void Run_Task_3(uint32_t current_time_ms)
{
    static uint8_t step = 0;
    static uint32_t led_timer = 0;
    static uint32_t stable_timer = 0;

    float current_err = 0.0f;

    switch (step)
    {
        // -------------------------------------------------
        // step 0：A -> C，高速斜穿
        // -------------------------------------------------
        case 0:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_A_TO_C;
            g_base_speed = TASK3_DIAG_SPEED;

            if (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_FAST_CM))
            {
                stable_timer = current_time_ms;
                step = 1;
            }
            break;

        // -------------------------------------------------
        // step 1：A -> C 后，车头修正到 0°，准备直走寻线
        // -------------------------------------------------
        case 1:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_C_SEEK;
            g_base_speed = TASK3_ALIGN_SPEED;

            current_err = Task_Get_Yaw_Error(TASK3_YAW_C_SEEK, g_yaw);

            if (fabs(current_err) > 5.0f)
            {
                stable_timer = current_time_ms;
            }

            // 车头稳定朝 0° 后，进入直走寻线
            if (current_time_ms - stable_timer >= 120)
            {
                step = 2;
            }
            break;

        // -------------------------------------------------
        // step 2：朝 0° 直走寻 C 点弧线
        // -------------------------------------------------
        case 2:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_C_SEEK;
            g_base_speed = TASK3_SEEK_SPEED;

            if ((Vision_Check_Timeout() == 1) ||
                (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_MAX_CM)))
            {
                g_base_speed = 0.0f;
                g_vision_enable = 0;
                LED_ON();

                led_timer = current_time_ms;
                step = 3;
            }
            break;

        // -------------------------------------------------
        // step 3：C 点提示，清零里程
        // -------------------------------------------------
        case 3:
            g_base_speed = 0.0f;
            g_vision_enable = 0;

            if (current_time_ms - led_timer >= 400)
            {
                LED_OFF();
                g_odom_distance_count = 0;
                step = 4;
            }
            break;

        // -------------------------------------------------
        // step 4：C -> B，视觉巡线
        // -------------------------------------------------
        case 4:
            g_vision_enable = 1;
            g_main_target_yaw = g_yaw;
            g_base_speed = TASK3_ARC_SPEED;

            if ((g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_MIN_CM) && TASK3_BACK_YAW_OK()) ||
                (g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_MAX_CM)))
            {
                g_vision_enable = 0;
                g_base_speed = 0.0f;
                LED_ON();

                led_timer = current_time_ms;
                step = 5;
            }
            break;

        // -------------------------------------------------
        // step 5：B 点提示
        // -------------------------------------------------
        case 5:
            g_base_speed = 0.0f;
            g_vision_enable = 0;

            if (current_time_ms - led_timer >= 400)
            {
                LED_OFF();
                g_odom_distance_count = 0;
                stable_timer = current_time_ms;
                step = 6;
            }
            break;

        // -------------------------------------------------
        // step 6：B 点车头先稳定到 -180°
        // -------------------------------------------------
        case 6:
            g_vision_enable = 0;
            g_main_target_yaw = -180.0f;
            g_base_speed = 0.0f;

            current_err = Task_Get_Yaw_Error(-180.0f, g_yaw);

            if (fabs(current_err) > 5.0f)
            {
                stable_timer = current_time_ms;
            }

            if (current_time_ms - stable_timer >= 120)
            {
                g_turn_offset = 0.0f;
                g_odom_distance_count = 0;
                step = 7;
            }
            break;

        // -------------------------------------------------
        // step 7：B -> D，高速斜穿
        // -------------------------------------------------
        case 7:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_B_TO_D;
            g_base_speed = TASK3_DIAG_SPEED;

            if (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_FAST_CM))
            {
                stable_timer = current_time_ms;
                step = 8;
            }
            break;

        // -------------------------------------------------
        // step 8：B -> D 后，车头修正到 -180°，准备直走寻线
        // -------------------------------------------------
        case 8:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_D_SEEK;
            g_base_speed = TASK3_ALIGN_SPEED;

            current_err = Task_Get_Yaw_Error(TASK3_YAW_D_SEEK, g_yaw);

            if (fabs(current_err) > 5.0f)
            {
                stable_timer = current_time_ms;
            }

            if (current_time_ms - stable_timer >= 120)
            {
                step = 9;
            }
            break;

        // -------------------------------------------------
        // step 9：朝 -180° 直走寻 D 点弧线
        // -------------------------------------------------
        case 9:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_D_SEEK;
            g_base_speed = TASK3_SEEK_SPEED;

            if ((Vision_Check_Timeout() == 1) ||
                (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_MAX_CM)))
            {
                g_base_speed = 0.0f;
                g_vision_enable = 0;
                LED_ON();

                led_timer = current_time_ms;
                step = 10;
            }
            break;

        // -------------------------------------------------
        // step 10：D 点提示，清零里程
        // -------------------------------------------------
        case 10:
            g_base_speed = 0.0f;
            g_vision_enable = 0;

            if (current_time_ms - led_timer >= 400)
            {
                LED_OFF();
                g_odom_distance_count = 0;
                step = 11;
            }
            break;

        // -------------------------------------------------
        // step 11：D -> A，视觉巡线
        // -------------------------------------------------
        case 11:
            g_vision_enable = 1;
            g_main_target_yaw = g_yaw;
            g_base_speed = TASK3_ARC_SPEED;

            if ((g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_MIN_CM) && TASK3_FRONT_YAW_OK()) ||
                (g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_MAX_CM)))
            {
                g_vision_enable = 0;
                g_base_speed = 0.0f;
                LED_ON();

                led_timer = current_time_ms;
                step = 12;
            }
            break;

        // -------------------------------------------------
        // step 12：A 点终点提示
        // -------------------------------------------------
        case 12:
            g_base_speed = 0.0f;
            g_vision_enable = 0;

            if (current_time_ms - led_timer >= 500)
            {
                LED_OFF();
                step = 13;
            }
            break;

        // -------------------------------------------------
        // step 13：任务结束
        // -------------------------------------------------
        case 13:
            g_base_speed = 0.0f;
            g_vision_enable = 0;
            break;
    }
}




// =========================================================================
// 🚀 任务4：任务3路径循环4圈
// 高速斜穿 + 回正直走寻线容错版
// =========================================================================
static void Run_Task_4(uint32_t current_time_ms)
{
    static uint8_t step = 0;
    static uint8_t lap_count = 0;
    static uint32_t led_timer = 0;
    static uint32_t stable_timer = 0;

    float current_err = 0.0f;

    switch (step)
    {
        case 0:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_A_TO_C;
            g_base_speed = TASK3_DIAG_SPEED;

            if (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_FAST_CM))
            {
                stable_timer = current_time_ms;
                step = 1;
            }
            break;

        case 1:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_C_SEEK;
            g_base_speed = TASK3_ALIGN_SPEED;

            current_err = Task_Get_Yaw_Error(TASK3_YAW_C_SEEK, g_yaw);

            if (fabs(current_err) > 5.0f)
            {
                stable_timer = current_time_ms;
            }

            if (current_time_ms - stable_timer >= 120)
            {
                step = 2;
            }
            break;

        case 2:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_C_SEEK;
            g_base_speed = TASK3_SEEK_SPEED;

            if ((Vision_Check_Timeout() == 1) ||
                (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_MAX_CM)))
            {
                g_base_speed = 0.0f;
                g_vision_enable = 0;
                LED_ON();

                led_timer = current_time_ms;
                step = 3;
            }
            break;

        case 3:
            g_base_speed = 0.0f;
            g_vision_enable = 0;

            if (current_time_ms - led_timer >= 300)
            {
                LED_OFF();
                g_odom_distance_count = 0;
                step = 4;
            }
            break;

        case 4:
            g_vision_enable = 1;
            g_main_target_yaw = g_yaw;
            g_base_speed = TASK3_ARC_SPEED;

            if ((g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_MIN_CM) && TASK3_BACK_YAW_OK()) ||
                (g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_MAX_CM)))
            {
                g_vision_enable = 0;
                g_base_speed = 0.0f;
                LED_ON();

                led_timer = current_time_ms;
                step = 5;
            }
            break;

        case 5:
            g_base_speed = 0.0f;
            g_vision_enable = 0;

            if (current_time_ms - led_timer >= 300)
            {
                LED_OFF();
                g_odom_distance_count = 0;
                stable_timer = current_time_ms;
                step = 6;
            }
            break;

        case 6:
            g_vision_enable = 0;
            g_main_target_yaw = -180.0f;
            g_base_speed = 0.0f;

            current_err = Task_Get_Yaw_Error(-180.0f, g_yaw);

            if (fabs(current_err) > 5.0f)
            {
                stable_timer = current_time_ms;
            }

            if (current_time_ms - stable_timer >= 120)
            {
                g_turn_offset = 0.0f;
                g_odom_distance_count = 0;
                step = 7;
            }
            break;

        case 7:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_B_TO_D;
            g_base_speed = TASK3_DIAG_SPEED;

            if (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_FAST_CM))
            {
                stable_timer = current_time_ms;
                step = 8;
            }
            break;

        case 8:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_D_SEEK;
            g_base_speed = TASK3_ALIGN_SPEED;

            current_err = Task_Get_Yaw_Error(TASK3_YAW_D_SEEK, g_yaw);

            if (fabs(current_err) > 5.0f)
            {
                stable_timer = current_time_ms;
            }

            if (current_time_ms - stable_timer >= 120)
            {
                step = 9;
            }
            break;

        case 9:
            g_vision_enable = 0;
            g_main_target_yaw = TASK3_YAW_D_SEEK;
            g_base_speed = TASK3_SEEK_SPEED;

            if ((Vision_Check_Timeout() == 1) ||
                (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_MAX_CM)))
            {
                g_base_speed = 0.0f;
                g_vision_enable = 0;
                LED_ON();

                led_timer = current_time_ms;
                step = 10;
            }
            break;

        case 10:
            g_base_speed = 0.0f;
            g_vision_enable = 0;

            if (current_time_ms - led_timer >= 300)
            {
                LED_OFF();
                g_odom_distance_count = 0;
                step = 11;
            }
            break;

        case 11:
            g_vision_enable = 1;
            g_main_target_yaw = g_yaw;
            g_base_speed = TASK3_ARC_SPEED;

            if ((g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_MIN_CM) && TASK3_FRONT_YAW_OK()) ||
                (g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_MAX_CM)))
            {
                g_vision_enable = 0;
                g_base_speed = 0.0f;
                LED_ON();

                led_timer = current_time_ms;
                step = 12;
            }
            break;

        case 12:
            g_base_speed = 0.0f;
            g_vision_enable = 0;

            if (current_time_ms - led_timer >= 300)
            {
                LED_OFF();

                lap_count++;

                if (lap_count < 4)
                {
                    g_odom_distance_count = 0;
                    stable_timer = current_time_ms;
                    step = 0;
                }
                else
                {
                    step = 13;
                }
            }
            break;

        case 13:
            g_base_speed = 0.0f;
            g_vision_enable = 0;
            break;
    }
}