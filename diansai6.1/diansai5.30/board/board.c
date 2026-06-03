
#include <stm32f4xx.h>
#include <board.h>

__IO uint32_t g_system_tick = 0;


/**
 * This function will initial stm32 board.
 */
 


void board_init(void)
{
    /* NVIC Configuration */
#define NVIC_VTOR_MASK              0x3FFFFF80
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x10000000 */
    SCB->VTOR  = (0x10000000 & NVIC_VTOR_MASK);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    SCB->VTOR  = (0x08000000 & NVIC_VTOR_MASK);
#endif

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);//ä½¿ç”¨168MHZå…¨é€Ÿæ—¶é’Ÿ
		//SysTick->LOAD=0xFFFF; 
	//SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; 
	/* é‡æ–°é…ç½® SysTick --- */
    // è¿™è¡Œä»£ç ä¼šè‡ªåŠ¨è®¾ç½® LOAD å¯„å­˜å™¨ã€æ¸…ç©º VAL å¯„å­˜å™¨ï¼Œå¹¶å¼€å¯ 1ms ä¸­æ–­
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1); // å¦‚æœé…ç½®å¤±è´¥åˆ™æ­»å¾ªç¯
    }
	
//	RCC_ClocksTypeDef rcc;
//	RCC_GetClocksFreq(&rcc);

}
// 2. æ·»åŠ  get_tick å‡½æ•°ä¾›å¤–éƒ¨è°ƒç”¨
uint32_t get_tick(void)
{
    return g_system_tick;
}

/**
 -  @brief  ÓÃÄÚºËµÄ systick ÊµÏÖµÄÎ¢ÃîÑÓÊ±
 -  @note   None
 -  @param  _us:ÒªÑÓÊ±µÄusÊı
 -  @retval None
*/
void delay_us(uint32_t _us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;

    // ¼ÆËãĞèÒªµÄÊ±ÖÓÊı = ÑÓ³ÙÎ¢ÃëÊı * Ã¿Î¢ÃëµÄÊ±ÖÓÊı
    ticks = _us * (SystemCoreClock / 1000000);

    // »ñÈ¡µ±Ç°µÄSysTickÖµ
    told = SysTick->VAL;

    while (1)
    {
        // ÖØ¸´Ë¢ĞÂ»ñÈ¡µ±Ç°µÄSysTickÖµ
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // Èç¹û´ïµ½ÁËĞèÒªµÄÊ±ÖÓÊı£¬¾ÍÍË³öÑ­»·
            if (tcnt >= ticks)
                break;
        }
    }
}

/**
 -  @brief  µ÷ÓÃÓÃÄÚºËµÄ systick ÊµÏÖµÄºÁÃëÑÓÊ±
 -  @note   None
 -  @param  _ms:ÒªÑÓÊ±µÄmsÊı
 -  @retval None
*/
void delay_ms(uint32_t _ms) { delay_us(_ms * 1000); }

void delay_1ms(uint32_t ms) { delay_us(ms * 1000); }

void delay_1us(uint32_t us) { delay_us(us); }
