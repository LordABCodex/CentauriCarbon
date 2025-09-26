#ifndef __STM32_HX717_EXIT_H
#define __STM32_HX717_EXIT_H
#include "internal.h" // GPIO

extern volatile int32_t g_hx711_exi_data[5]; 
extern volatile uint64_t g_hx711_exi_tick[5]; 
void HX717_init(uint32_t sensor_count);
void my_delay_us(uint32_t us);
void Stm32_Clock_Init(uint32_t plln,uint32_t pllm,uint32_t pllp,uint32_t pllq);
void my_NVIC_set_vectortable(uint32_t NVIC_VectTab,uint32_t Offset)	 ;
void Test(void);
#endif