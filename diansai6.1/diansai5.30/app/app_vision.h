#ifndef __APP_VISION_H
#define __APP_VISION_H

#include <stdint.h>

// 视觉数据结构体：包含误差值与有效性标志位
typedef struct {
    float Error_X;        // 视觉计算出的横向偏移误差 (例如像素偏差)
    uint8_t Is_Valid;     // 数据有效性/丢帧标志位 (1:有效 0:丢帧或异常)
    uint32_t Last_Update; // 上一次收到有效数据的时间戳 (用于超时检测)
} Vision_Data_t;

// 在 app_vision.h 中修改声明：
extern Vision_Data_t g_vision_info;

void App_Vision_Init(void);
void Vision_Data_Parse(uint8_t *rx_buf, uint16_t len); // 给K230串口接收中断/DMA调用的解析函数
float Vision_Control_Loop(float vision_error);         // 视觉外环控制算子
uint8_t Vision_Check_Timeout(void);
#endif

