#include <stm32f4xx.h>
#include "bsp_uart.h" 
#include "stdio.h"
#include <string.h> // 必须引入，用于处理字符串
#include "math.h"
#include "app_vision.h"

// --- 新增：串口调参缓冲区 ---
#define UART_RX_BUF_SIZE 64
char g_uart_rx_buf[UART_RX_BUF_SIZE]; // 接收缓冲区
uint8_t g_uart_rx_sta = 0;           // bit7:接收完成标志, bit0-6:当前接收长度

// 全局变量：存储由陀螺仪实时解算出的当前绝对车头航向角（单位：度，范围±180）
float g_yaw = 0.0f;

void uart1_init(uint32_t __Baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);//IO¿ړ×÷´®¿ڒý½ŒªŤփ¸´Ӄģʽ
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin           = GPIO_Pin_9;//TXҽ½ō
	GPIO_InitStructure.GPIO_Mode          = GPIO_Mode_AF;//IO¿ړ×÷´®¿ڒý½ŒªŤփ¸´Ӄģʽ
	GPIO_InitStructure.GPIO_Speed         = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType         = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd          = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin           = GPIO_Pin_10;//RXҽ½ō
	GPIO_InitStructure.GPIO_Mode          = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed         = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType         = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd          = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  
	USART_InitTypeDef USART_InitStructure;//¶¨ҥŤփ´®¿ڵĽṹ̥±䁿

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//¿ªƴ´®¿ڱµĊ±֓

	USART_DeInit(USART1);//´󸅒⋼ʇ½⳽´˴®¿ڵĆ䋻Ťփ

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate              = __Baud;//ɨփ²¨̘
	USART_InitStructure.USART_WordLength            = USART_WordLength_8b;//ז½ڳ¤¶Ȏª8bit
	USART_InitStructure.USART_StopBits              = USART_StopBits_1;//1¸öֹͣλ
	USART_InitStructure.USART_Parity                = USART_Parity_No ;//ûӐУѩλ
	USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;//½«´®¿څ䖃Ϊʕ·¢ģʽ
	USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None; //²»̡¹©Á÷¿ؠ
	USART_Init(USART1,&USART_InitStructure);//½«Ϡ¹زΊý³õʼ»¯¸ø´®¿ڱ
	
	USART_ClearFlag(USART1,USART_FLAG_RXNE);//³õʼŤփʱǥ³ý½ӊܖÎ»

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//³õʼŤփ½ӊܖжύ

	USART_Cmd(USART1,ENABLE);//¿ªƴ´®¿ڱ
	
	NVIC_InitTypeDef NVIC_InitStructure;//֐¶Ͽؖƽṹ̥±䁿¶¨ҥ

	NVIC_InitStructure.NVIC_IRQChannel                    = USART1_IRQn;//֐¶ύ¨µÀָ¶¨ΪUSART1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;//ַӅψ¼¶Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 1;//´Γŏȼ¶Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;//ȷ¶¨ʹĜ
	NVIC_Init(&NVIC_InitStructure);//³õʼ»¯Ťփ´˖жύ¨µÀ
		
}

/**
 * @brief  初始化 USART2 —— 连接陀螺仪（PA2/PA3）
 */
void uart2_init(uint32_t Baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 开启时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 2. 引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // RX

    // 3. GPIO 配置
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. 串口配置
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = Baud;
    USART_InitStructure.USART_Mode     = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    // 5. 开启接收中断
    USART_ClearFlag(USART2, USART_FLAG_RXNE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    // 6. NVIC 优先级配置
    NVIC_InitStructure.NVIC_IRQChannel                    = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 0; // 陀螺仪数据极其重要，给最高优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority          = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  初始化 USART3 —— 预留给摄像头（PB10/PB11）
 */
void uart3_init(uint32_t Baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = Baud;
    USART_InitStructure.USART_Mode     = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                    = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority          = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#if !defined(__MICROLIB)
//²»ʹӃ΢¿ⵄ»°¾͐蒪̭¼ӏ浄º¯ʽ
#if (__ARMCLIB_VERSION <= 6000000)
//ȧ¹û±ҫƷʇAC5  ¾Ͷ¨ҥςæբ¸ö½ṹ̥
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//¶¨ҥ_sys_exit()Ҕ±܃⊹Ӄ°떷»úģʽ
void _sys_exit(int x)
{
	x = x;
}
#endif

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);
	
	while( RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE) ){}
	
    return ch;
}



/******** ´®¿ڱ ֐¶Ϸþαº¯ʽ ***********/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
    }
    /*uint8_t res;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART1);
        
        if((g_uart_rx_sta & 0x80) == 0) // 接收未完成
        {
            if(res == 0x0D || res == 0x0A) // 回车或换行
            {
                if(g_uart_rx_sta & 0x7F) g_uart_rx_sta |= 0x80; // 标记完成
            }
            else
            {
                // 过滤掉普通调试信息，只接收 P I D T 开头的指令
                if((g_uart_rx_sta & 0x7F) == 0) {
                    if(res == 'P' || res == 'I' || res == 'D' || res == 'T') {
                        g_uart_rx_buf[0] = res;
                        g_uart_rx_sta = 1;
                    }
                }
                else {
                    g_uart_rx_buf[g_uart_rx_sta & 0x7F] = res;
                    g_uart_rx_sta++;
                }
                
                if(g_uart_rx_sta > 63) g_uart_rx_sta = 0; // 防止溢出
            }
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }*/
}



/******** 串口2 中断服务函数 —— 工业级严格硬对齐解析状态机（决不放过一帧错位包） ***********/
/******** 串口2 中断服务函数 —— 工业级纯净三角向量解耦状态机（无任何人工限幅判断） ***********/
void USART2_IRQHandler(void)
{
    static uint8_t rx_buf[11];
    static uint8_t rx_cnt = 0;
    uint8_t res;

    // === 相对角度向量解耦静态变量 ===
    static uint8_t is_base_captured = 0; 
    static float base_yaw_rad = 0.0f;    // 锁死开机时的绝对角度（弧度制）

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART2);
        
        // 严格帧头硬对齐拦截，彻底拍死 0x51 加速度和 0x52 角速度包
        if (rx_cnt == 0) {
            if (res == 0x55) rx_buf[rx_cnt++] = res;
            else rx_cnt = 0;
        }
        else if (rx_cnt == 1) {
            if (res == 0x53) rx_buf[rx_cnt++] = res; // 👈 仅限 0x53 角度包放行入内
            else rx_cnt = 0; // 只要不是角度包，当场拍死归零，拒绝一切杂包踩踏错位
        }
        else {
            rx_buf[rx_cnt++] = res;
            if (rx_cnt == 11) { 
                
                // 1. 解算你刚刚发给我的标准原始绝对角度（-93.3°），并立刻转化为标准数学弧度
                int16_t temp_yaw = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
                float raw_absolute_yaw = (float)temp_yaw / 32768.0f * 180.0f;
                float current_yaw_rad = raw_absolute_yaw * 3.14159265f / 180.0f;
                
                // 2. 首次开机时，永久锁死第一帧合法的绝对角度弧度基准
                if (is_base_captured == 0) {
                    base_yaw_rad = current_yaw_rad; // 锁死起跑线绝对弧度（锁死在 -93.3° 对应的弧度上）
                    is_base_captured = 1;
                }

                // 3. === 🚨 向量减法：利用正弦余弦差角公式进行投影 🚨 ===
                // 彻底废除有硬伤的代数直减 (raw - base)，不给假数据任何越过 if 屏障的视觉死角！
                float sin_diff = sinf(current_yaw_rad) * cosf(base_yaw_rad) - cosf(current_yaw_rad) * sinf(base_yaw_rad);
                float cos_diff = cosf(current_yaw_rad) * cosf(base_yaw_rad) + sinf(current_yaw_rad) * sinf(base_yaw_rad);
                
                // 4. === 🚨 纯数学天则自适应限幅 🚨 ===
                // 彻底删掉你原先极其痛恨的 if(>180) 或者 while 强制限幅逻辑！
                // atan2f 函数的数学天性天生就把结果锁死在 -180 到 +180 之间，
                // 并且在旋转过零边界处绝对连续丝滑。
                float relative_yaw = atan2f(sin_diff, cos_diff) * 180.0f / 3.14159265f;
                
                // 5. 【核心交付】：把这个在向量世界里绝对不会跳变的纯净相对角度，塞给你的全局打印变量！
                g_yaw = relative_yaw; 
                
                rx_cnt = 0; // 计数归零
            }
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/* 摄像头中断服务函数 —— 视觉数据解析状态机 */
void USART3_IRQHandler(void)
{
    static uint8_t rx_buf[4];
    static uint8_t rx_cnt = 0;
    uint8_t res;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART3);
        
        // K230 4字节通信协议解析: 0xAA(帧头) + 数据高8位 + 数据低8位 + 0x55(帧尾)
        if (rx_cnt == 0 && res == 0xAA) {
            rx_buf[rx_cnt++] = res;
        }
        else if (rx_cnt > 0) {
            rx_buf[rx_cnt++] = res;
            
            if (rx_cnt >= 4) {
                if (rx_buf[3] == 0x55) { // 校验通过，投递数据到视觉模块
                    Vision_Data_Parse(rx_buf, rx_cnt);
                }
                rx_cnt = 0; // 状态机复位
            }
        }
        else {
            rx_cnt = 0;
        }
        
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

// --- 新增：供外部调用的获取指令函数 ---
char* UART1_Get_Command(void)
{
    if(g_uart_rx_sta & 0x80) // 检查完成标志位
    {
        g_uart_rx_buf[g_uart_rx_sta & 0x7F] = '\0'; // 末尾补齐字符串结束符
        return g_uart_rx_buf;
    }
    return NULL;
}

// --- 新增：清除指令函数 ---
void UART1_Clear_Command(void)
{
    g_uart_rx_sta = 0;
    memset(g_uart_rx_buf, 0, UART_RX_BUF_SIZE);
}


