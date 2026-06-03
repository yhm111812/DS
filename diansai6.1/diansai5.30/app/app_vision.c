#include "app_vision.h"
#include "bsp_timer.h" // 获取系统时间，用于超时检测
#include "board.h"
#include "math.h"

Vision_Data_t g_vision_info = {0.0f, 0, 0};

/* 视觉环(最外环)高级结构体 - 依照原有代码风格隔离 */
typedef struct {
    float Target;          // 期望偏移(巡线通常为0)
    float Kp, Ki, Kd;      
    float Error, Last_Error;
    float Output_Max;      // 最大输出角度限制 (限制车头单次修正的最大偏角)
} Vision_PID_t;

// 视觉环初始化：Kp需要根据你的像素偏差范围去推算，这里给一个参考值。
// 限制视觉最大只能调整车头正负 30 度，防止跑飞
Vision_PID_t pid_Vision = {0.0f, 0.05f, 0.0f, 0.02f, 0.0f, 0.0f, 30.0f}; 

void App_Vision_Init(void)
{
    g_vision_info.Error_X = 0.0f;
    g_vision_info.Is_Valid = 0;
    g_vision_info.Last_Update = 0;
}

/**
 * @brief  检查视觉数据是否超时丢帧
 * @retval 1: 数据有效且未超时, 0: 超时丢帧
 */
uint8_t Vision_Check_Timeout(void)
{
    // main.c 主循环设定为 200ms 超时判定
    if (get_tick() - g_vision_info.Last_Update > 200) {
        g_vision_info.Is_Valid = 0;
        return 0; // 发生超时丢帧
    }
    return 1; // 数据有效
}

/**
 * @brief  【视觉解析函数】由K230专属串口中断或DMA回调调用
 * @note   通信格式：0xAA(帧头) + 数据高8位 + 数据低8位 + 0x55(帧尾)
 */
void Vision_Data_Parse(uint8_t *rx_buf, uint16_t len)
{
    if (len >= 4 && rx_buf[0] == 0xAA && rx_buf[3] == 0x55) {
        // 将高低8位拼接成16位有符号整数（假设传入的是带符号的像素偏差）
        int16_t error_temp = (int16_t)((rx_buf[1] << 8) | rx_buf[2]);
        
        g_vision_info.Error_X = (float)error_temp;
        g_vision_info.Is_Valid = 1;
        
        // 【关键修复】刷新心跳时间戳，避免被判定为一直处于断开状态
        g_vision_info.Last_Update = get_tick(); 
    }
}

/**
 * @brief  【最外层：视觉环】PID控制函数
 * @param  vision_error 当前视觉计算出的画面偏移量
 * @return 吐出需要补偿给角度环的“目标航向角偏置” (度)
 */
float Vision_Control_Loop(float vision_error)
{
    // 丢帧保护统一交由 main.c 处理，此处专注纯控制算法，避免逻辑冲突

    pid_Vision.Target = 0.0f; // 巡线目标：误差归零
    pid_Vision.Error = pid_Vision.Target - vision_error;
    
    // 视觉一般采用 PD 控制。D的作用是预测路线弯曲趋势，极大地减少车头画龙(震荡)
    float vision_output = (pid_Vision.Kp * pid_Vision.Error) + 
                          (pid_Vision.Kd * (pid_Vision.Error - pid_Vision.Last_Error));
    
    // 安全限幅：绝不允许视觉给出破坏性的角度指令
    if(vision_output > pid_Vision.Output_Max)  vision_output = pid_Vision.Output_Max;
    if(vision_output < -pid_Vision.Output_Max) vision_output = -pid_Vision.Output_Max;
    
    pid_Vision.Last_Error = pid_Vision.Error;
    
    return vision_output; 
}

