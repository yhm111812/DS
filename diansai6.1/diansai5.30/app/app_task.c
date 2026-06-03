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

// 斜穿最高速度仍然是 15.0f
#define TASK3_DIAG_SPEED        (30.0f)

// 斜穿起步速度，避免一下子冲出去
#define TASK3_DIAG_START_SPEED  (13.0f)

// 斜穿末端速度，准备回正前先降速
#define TASK3_DIAG_END_SPEED    (8.0f)

// 回正速度不要太快，越快越晃
#define TASK3_ALIGN_SPEED       (1.8f)

// 寻线速度
#define TASK3_SEEK_SPEED        (3.0f)

// 弯道速度
#define TASK3_ARC_SPEED         (30.0f)

// 斜穿高速距离
#define TASK3_DIAG_FAST_CM      (120.0f)


// 斜穿起步加速距离
#define TASK3_DIAG_ACCEL_CM     (25.0f)

// 斜穿末端减速距离
#define TASK3_DIAG_DECEL_CM     (20.0f)

// 回正时目标角每 20ms 最多变化多少度
// 越小越稳，越大越快
#define TASK3_YAW_RAMP_STEP     (1.2f)

// 回正允许误差
#define TASK3_ALIGN_ERR_DEG     (4.0f)

// 回正稳定时间
#define TASK3_ALIGN_STABLE_MS   (180)



// 距离
// 因为斜穿速度变成 15，不能再跑满 128cm，否则必然冲过头

#define TASK3_DIAG_MAX_CM       (135.0f)  // 最多寻线距离，防死等

// 弯道判定
#define TASK3_ARC_MIN_CM        (120.0f)    // 过了这个距离才允许用 yaw 判断到点
#define TASK3_ARC_FORCE_CM      (125.0f)   // 到这个距离直接强制认为到点
#define TASK3_ARC_MAX_CM        (130.0f)   // 最终保护距离，不建议再用 138


// yaw 判断
#define TASK3_BACK_YAW_OK()     ((g_yaw >= 145.0f) || (g_yaw <= -145.0f))
#define TASK3_FRONT_YAW_OK()    (fabs(g_yaw) <= 24.0f)

#define TASK3_START_ENTER_SPEED     (1.5f)
#define TASK3_START_YAW_STEP        (0.3f)
#define TASK3_START_ERR_DEG         (3.0f)
#define TASK3_START_STABLE_MS       (180)


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
    g_current_mode = MODE_TASK_4; 
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

/**
 * @brief  目标角度缓慢过渡，避免目标角硬跳导致车身大幅摆动
 */
static float Task_Ramp_Yaw_Target(float current_target, float final_target, float max_step)
{
    float err = Task_Get_Yaw_Error(final_target, current_target);

    if (err > max_step)
    {
        current_target += max_step;
    }
    else if (err < -max_step)
    {
        current_target -= max_step;
    }
    else
    {
        current_target = final_target;
    }

    while (current_target > 180.0f)  current_target -= 360.0f;
    while (current_target < -180.0f) current_target += 360.0f;

    return current_target;
}

/**
 * @brief  斜穿速度曲线：起步渐升，中段 15.0f，末端降速
 */
static float Task_Diag_Speed_Profile(void)
{
    float dist_cm = (float)g_odom_distance_count / 66.6f;
    float speed = TASK3_DIAG_SPEED;

    // 起步阶段：6.0 -> 15.0
    if (dist_cm < TASK3_DIAG_ACCEL_CM)
    {
        speed = TASK3_DIAG_START_SPEED +
                (TASK3_DIAG_SPEED - TASK3_DIAG_START_SPEED) *
                (dist_cm / TASK3_DIAG_ACCEL_CM);
    }
    // 末端阶段：15.0 -> 8.0
    else if (dist_cm > (TASK3_DIAG_FAST_CM - TASK3_DIAG_DECEL_CM))
    {
        float decel_pos = dist_cm - (TASK3_DIAG_FAST_CM - TASK3_DIAG_DECEL_CM);

        speed = TASK3_DIAG_SPEED -
                (TASK3_DIAG_SPEED - TASK3_DIAG_END_SPEED) *
                (decel_pos / TASK3_DIAG_DECEL_CM);

        if (speed < TASK3_DIAG_END_SPEED)
        {
            speed = TASK3_DIAG_END_SPEED;
        }
    }
    else
    {
        speed = TASK3_DIAG_SPEED;
    }

    return speed;
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

{
					static uint8_t start_phase_ac = 0;
					static float ramp_yaw_ac = 0.0f;
					static uint8_t ramp_init_ac = 0;

					g_vision_enable = 0;

					// phase 0：起步缓慢把目标角从当前yaw过渡到A->C斜穿角
					if (start_phase_ac == 0)
					{
							if (ramp_init_ac == 0)
							{
									ramp_yaw_ac = g_yaw;
									ramp_init_ac = 1;

									g_turn_offset = 0.0f;
									g_odom_distance_count = 0;
									stable_timer = current_time_ms;
							}

							ramp_yaw_ac = Task_Ramp_Yaw_Target(ramp_yaw_ac,
																								 TASK3_YAW_A_TO_C,
																								 TASK3_START_YAW_STEP);

							g_main_target_yaw = ramp_yaw_ac;
							g_base_speed = TASK3_START_ENTER_SPEED;

							current_err = Task_Get_Yaw_Error(TASK3_YAW_A_TO_C, g_yaw);

							if (fabs(current_err) > TASK3_START_ERR_DEG)
							{
									stable_timer = current_time_ms;
							}

							if (current_time_ms - stable_timer >= TASK3_START_STABLE_MS)
							{
									g_turn_offset = 0.0f;
									g_odom_distance_count = 0;
									start_phase_ac = 1;
							}
					}
					// phase 1：正式A->C高速斜穿
					else
					{
							g_main_target_yaw = TASK3_YAW_A_TO_C;
							g_base_speed = Task_Diag_Speed_Profile();

							if (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_FAST_CM))
							{
									stable_timer = current_time_ms;

									start_phase_ac = 0;
									ramp_init_ac = 0;
									step = 1;
							}
					}

					break;
}

        // -------------------------------------------------
        // step 1：A -> C 后，车头修正到 0°，准备直走寻线
        // -------------------------------------------------
        case 1:
					{
							static float ramp_yaw_c = 0.0f;
							static uint8_t ramp_init_c = 0;

							g_vision_enable = 0;
							g_base_speed = TASK3_ALIGN_SPEED;

							// 第一次进入回正阶段时，用当前 yaw 作为缓变起点
							if (ramp_init_c == 0)
							{
									ramp_yaw_c = g_yaw;
									ramp_init_c = 1;
							}

							// 目标角慢慢逼近 0°
							ramp_yaw_c = Task_Ramp_Yaw_Target(ramp_yaw_c,
																								TASK3_YAW_C_SEEK,
																								TASK3_YAW_RAMP_STEP);

							g_main_target_yaw = ramp_yaw_c;

							current_err = Task_Get_Yaw_Error(TASK3_YAW_C_SEEK, g_yaw);

							if (fabs(current_err) > TASK3_ALIGN_ERR_DEG)
							{
									stable_timer = current_time_ms;
							}

							if (current_time_ms - stable_timer >= TASK3_ALIGN_STABLE_MS)
							{
									ramp_init_c = 0;
									step = 2;
							}
							break;
}

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
									(g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_FORCE_CM)))
							{
									g_vision_enable = 0;
									g_base_speed = 0.0f;
									g_turn_offset = 0.0f;
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
{
						static uint8_t start_phase_bd = 0;
						static float ramp_yaw_bd = 0.0f;
						static uint8_t ramp_init_bd = 0;

						g_vision_enable = 0;

						// phase 0：从当前yaw缓慢过渡到B->D斜穿角
						if (start_phase_bd == 0)
						{
								if (ramp_init_bd == 0)
								{
										ramp_yaw_bd = g_yaw;
										ramp_init_bd = 1;

										g_turn_offset = 0.0f;
										g_odom_distance_count = 0;
										stable_timer = current_time_ms;
								}

								ramp_yaw_bd = Task_Ramp_Yaw_Target(ramp_yaw_bd,
																									 TASK3_YAW_B_TO_D,
																									 TASK3_START_YAW_STEP);

								g_main_target_yaw = ramp_yaw_bd;
								g_base_speed = TASK3_START_ENTER_SPEED;

								current_err = Task_Get_Yaw_Error(TASK3_YAW_B_TO_D, g_yaw);

								if (fabs(current_err) > TASK3_START_ERR_DEG)
								{
										stable_timer = current_time_ms;
								}

								if (current_time_ms - stable_timer >= TASK3_START_STABLE_MS)
								{
										g_turn_offset = 0.0f;
										g_odom_distance_count = 0;
										start_phase_bd = 1;
								}
						}
						// phase 1：正式B->D高速斜穿
						else
						{
								g_main_target_yaw = TASK3_YAW_B_TO_D;
								g_base_speed = Task_Diag_Speed_Profile();

								if (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_FAST_CM))
								{
										stable_timer = current_time_ms;

										start_phase_bd = 0;
										ramp_init_bd = 0;
										step = 8;
								}
						}

						break;
}

        // -------------------------------------------------
        // step 8：B -> D 后，车头修正到 -180°，准备直走寻线
        // -------------------------------------------------
        case 8:
{
							static float ramp_yaw_d = 0.0f;
							static uint8_t ramp_init_d = 0;

							g_vision_enable = 0;
							g_base_speed = TASK3_ALIGN_SPEED;

							// 第一次进入 D 点回正阶段时，用当前 yaw 作为缓变起点
							if (ramp_init_d == 0)
							{
									ramp_yaw_d = g_yaw;
									ramp_init_d = 1;
							}

							// 目标角慢慢逼近 -180°
							ramp_yaw_d = Task_Ramp_Yaw_Target(ramp_yaw_d,
																								TASK3_YAW_D_SEEK,
																								TASK3_YAW_RAMP_STEP);

							g_main_target_yaw = ramp_yaw_d;

							current_err = Task_Get_Yaw_Error(TASK3_YAW_D_SEEK, g_yaw);

							if (fabs(current_err) > TASK3_ALIGN_ERR_DEG)
							{
									stable_timer = current_time_ms;
							}

							if (current_time_ms - stable_timer >= TASK3_ALIGN_STABLE_MS)
							{
									ramp_init_d = 0;
									step = 9;
							}
							break;
}

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
								(g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_FORCE_CM)))
						{
								g_vision_enable = 0;
								g_base_speed = 0.0f;
								g_turn_offset = 0.0f;
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
{
								static uint8_t start_phase_ac = 0;
								static float ramp_yaw_ac = 0.0f;
								static uint8_t ramp_init_ac = 0;

								g_vision_enable = 0;

								// phase 0：每圈从A点起步时，先缓慢进入A->C斜穿角
								if (start_phase_ac == 0)
								{
										if (ramp_init_ac == 0)
										{
												ramp_yaw_ac = g_yaw;
												ramp_init_ac = 1;

												g_turn_offset = 0.0f;
												g_odom_distance_count = 0;
												stable_timer = current_time_ms;
										}

										ramp_yaw_ac = Task_Ramp_Yaw_Target(ramp_yaw_ac,
																											 TASK3_YAW_A_TO_C,
																											 TASK3_START_YAW_STEP);

										g_main_target_yaw = ramp_yaw_ac;
										g_base_speed = TASK3_START_ENTER_SPEED;

										current_err = Task_Get_Yaw_Error(TASK3_YAW_A_TO_C, g_yaw);

										if (fabs(current_err) > TASK3_START_ERR_DEG)
										{
												stable_timer = current_time_ms;
										}

										if (current_time_ms - stable_timer >= TASK3_START_STABLE_MS)
										{
												g_turn_offset = 0.0f;
												g_odom_distance_count = 0;
												start_phase_ac = 1;
										}
								}
								// phase 1：正式A->C斜穿
								else
								{
										g_main_target_yaw = TASK3_YAW_A_TO_C;
										g_base_speed = Task_Diag_Speed_Profile();

										if (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_FAST_CM))
										{
												stable_timer = current_time_ms;

												start_phase_ac = 0;
												ramp_init_ac = 0;
												step = 1;
										}
								}

								break;
}

        case 1:
{
						static float ramp_yaw_c = 0.0f;
						static uint8_t ramp_init_c = 0;

						g_vision_enable = 0;
						g_base_speed = TASK3_ALIGN_SPEED;

						if (ramp_init_c == 0)
						{
								ramp_yaw_c = g_yaw;
								ramp_init_c = 1;
						}

						ramp_yaw_c = Task_Ramp_Yaw_Target(ramp_yaw_c,
																							TASK3_YAW_C_SEEK,
																							TASK3_YAW_RAMP_STEP);

						g_main_target_yaw = ramp_yaw_c;

						current_err = Task_Get_Yaw_Error(TASK3_YAW_C_SEEK, g_yaw);

						if (fabs(current_err) > TASK3_ALIGN_ERR_DEG)
						{
								stable_timer = current_time_ms;
						}

						if (current_time_ms - stable_timer >= TASK3_ALIGN_STABLE_MS)
						{
								ramp_init_c = 0;
								step = 2;
						}
						break;
}

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
								(g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_FORCE_CM)))
						{
								g_vision_enable = 0;
								g_base_speed = 0.0f;
								g_turn_offset = 0.0f;
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
{
							static uint8_t start_phase_bd = 0;
							static float ramp_yaw_bd = 0.0f;
							static uint8_t ramp_init_bd = 0;

							g_vision_enable = 0;

							// phase 0：从B点出发时，先缓慢进入B->D斜穿角
							if (start_phase_bd == 0)
							{
									if (ramp_init_bd == 0)
									{
											ramp_yaw_bd = g_yaw;
											ramp_init_bd = 1;

											g_turn_offset = 0.0f;
											g_odom_distance_count = 0;
											stable_timer = current_time_ms;
									}

									ramp_yaw_bd = Task_Ramp_Yaw_Target(ramp_yaw_bd,
																										 TASK3_YAW_B_TO_D,
																										 TASK3_START_YAW_STEP);

									g_main_target_yaw = ramp_yaw_bd;
									g_base_speed = TASK3_START_ENTER_SPEED;

									current_err = Task_Get_Yaw_Error(TASK3_YAW_B_TO_D, g_yaw);

									if (fabs(current_err) > TASK3_START_ERR_DEG)
									{
											stable_timer = current_time_ms;
									}

									if (current_time_ms - stable_timer >= TASK3_START_STABLE_MS)
									{
											g_turn_offset = 0.0f;
											g_odom_distance_count = 0;
											start_phase_bd = 1;
									}
							}
							// phase 1：正式B->D斜穿
							else
							{
									g_main_target_yaw = TASK3_YAW_B_TO_D;
									g_base_speed = Task_Diag_Speed_Profile();

									if (g_odom_distance_count >= CM_TO_PULSE(TASK3_DIAG_FAST_CM))
									{
											stable_timer = current_time_ms;

											start_phase_bd = 0;
											ramp_init_bd = 0;
											step = 8;
									}
							}

							break;
}

        case 8:
{
						static float ramp_yaw_d = 0.0f;
						static uint8_t ramp_init_d = 0;

						g_vision_enable = 0;
						g_base_speed = TASK3_ALIGN_SPEED;

						if (ramp_init_d == 0)
						{
								ramp_yaw_d = g_yaw;
								ramp_init_d = 1;
						}

						ramp_yaw_d = Task_Ramp_Yaw_Target(ramp_yaw_d,
																							TASK3_YAW_D_SEEK,
																							TASK3_YAW_RAMP_STEP);

						g_main_target_yaw = ramp_yaw_d;

						current_err = Task_Get_Yaw_Error(TASK3_YAW_D_SEEK, g_yaw);

						if (fabs(current_err) > TASK3_ALIGN_ERR_DEG)
						{
								stable_timer = current_time_ms;
						}

						if (current_time_ms - stable_timer >= TASK3_ALIGN_STABLE_MS)
						{
								ramp_init_d = 0;
								step = 9;
						}
						break;
}

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
								(g_odom_distance_count >= CM_TO_PULSE(TASK3_ARC_FORCE_CM)))
						{
								g_vision_enable = 0;
								g_base_speed = 0.0f;
								g_turn_offset = 0.0f;
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
										g_base_speed = 0.0f;
										g_vision_enable = 0;
										g_turn_offset = 0.0f;
										g_main_target_yaw = 0.0f;
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