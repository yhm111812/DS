#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "stm32f4xx.h"

// 运动方向枚举
typedef enum {
    MOTOR_REV = 0, // 反转
    MOTOR_FWD = 1  // 正转
} MotorDir_t;

void App_Motor_Init(void);
float Motor_Position_Control_Loop(float target_pos, float actual_pos);
float Motor_Yaw_Control_Loop(float target_yaw, float actual_yaw);
void Motor_Speed_Control_PID(void);

#endif
