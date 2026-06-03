#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "board.h" 

// 赛题任务模式枚举
typedef enum {
    MODE_WAIT = 0, // 待命状态
    MODE_TASK_1,   // 任务1：A -> B 停车
    MODE_TASK_2,   // 任务2：A -> B -> C -> D -> A 停车
    MODE_TASK_3,   // 任务3：A -> C -> B -> D -> A 停车 (交叉)
    MODE_TASK_4    // 任务4：任务3连跑4圈
} RaceMode_e;

// 暴露给外部 (如按键中断或 main) 来切换任务
extern RaceMode_e g_current_mode; 

// 暴露接口
void App_Task_Init(void);
void App_Task_Run(uint32_t current_time_ms);

#endif
