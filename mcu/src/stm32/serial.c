// STM32 serial
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"          // CONFIG_SERIAL_BAUD
#include "board/armcm_boot.h"  // armcm_enable_irq
#include "board/misc.h"
#include "board/serial_irq.h"  // serial_rx_byte
#include "board/stm32f4xx_gpio.h"
#include "board/stm32f4xx_rcc.h"
#include "board/stm32f4xx_usart.h"
#include "command.h"   // DECL_CONSTANT_STR
#include "internal.h"  // enable_pclock
#include "sched.h"     // DECL_INIT

// Select the configured serial port
#if CONFIG_STM32_SERIAL_USART1
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA10,PA9");
#define GPIO_Rx GPIO('A', 10)
#define GPIO_Tx GPIO('A', 9)
#define USARTx USART1
#define USARTx_IRQn USART1_IRQn
#elif CONFIG_STM32_SERIAL_USART1_ALT_PB7_PB6
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB7,PB6");
#define GPIO_Rx GPIO('B', 7)
#define GPIO_Tx GPIO('B', 6)
#define USARTx USART1
#define USARTx_IRQn USART1_IRQn
#elif CONFIG_STM32_SERIAL_USART2
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA3,PA2");
#define GPIO_Rx GPIO('A', 3)
#define GPIO_Tx GPIO('A', 2)
#define USARTx USART2
#define USARTx_IRQn USART2_IRQn
#elif CONFIG_STM32_SERIAL_USART2_ALT_PD6_PD5
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD6,PD5");
#define GPIO_Rx GPIO('D', 6)
#define GPIO_Tx GPIO('D', 5)
#define USARTx USART2
#define USARTx_IRQn USART2_IRQn
#elif CONFIG_STM32_SERIAL_USART3
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB11,PB10");
#define GPIO_Rx GPIO('B', 11)
#define GPIO_Tx GPIO('B', 10)
#define USARTx USART3
#define USARTx_IRQn USART3_IRQn
#elif CONFIG_STM32_SERIAL_USART3_ALT_PD9_PD8
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD9,PD8");
#define GPIO_Rx GPIO('D', 9)
#define GPIO_Tx GPIO('D', 8)
#define USARTx USART3
#define USARTx_IRQn USART3_IRQn
#endif

#define CR1_FLAGS \
  (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE)

void USARTx_IRQHandler(void) {
  uint32_t sr = USARTx->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
    // The ORE flag is automatically cleared by reading SR, followed
    // by reading DR.
    serial_rx_byte(USARTx->DR);
  }
  if (sr & USART_SR_TXE && USARTx->CR1 & USART_CR1_TXEIE) {
    uint8_t data;
    int ret = serial_get_tx_byte(&data);
    if (ret)
      USARTx->CR1 = CR1_FLAGS;
    else
      USARTx->DR = data;
  }
}

/**
 * @description: 串口1中断服务函数
 * @author: your name
 * @return {*}
 */
void USART1_IRQHandler(void) {
  uint32_t sr = USART1->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
    // The ORE flag is automatically cleared by reading SR, followed
    // by reading DR.
    // serial_rx_byte(USART1->DR);
    int ret = usart1_post(USART1->DR);
    // uint8_t data = USART1->DR;
  }
  if (sr & USART_SR_TXE && USART1->CR1 & USART_CR1_TXEIE) {
    uint8_t data;
    // int ret = serial_get_tx_byte(&data);
    int ret = 1;
    if (ret)
      USART1->CR1 = CR1_FLAGS;
    else
      USART1->DR = data;
  }
}

void serial_enable_tx_irq(void) { USARTx->CR1 = CR1_FLAGS | USART_CR1_TXEIE; }
void serial1_enable_tx_irq(void) { USART1->CR1 = CR1_FLAGS | USART_CR1_TXEIE; }

void serial_init(void) {
  enable_pclock((uint32_t)USARTx);

  uint32_t pclk = get_pclock_frequency((uint32_t)USARTx);
  uint32_t div = DIV_ROUND_CLOSEST(pclk, CONFIG_SERIAL_BAUD);
  USARTx->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos) |
                 ((div % 16) << USART_BRR_DIV_Fraction_Pos));
  USARTx->CR1 = CR1_FLAGS;
  armcm_enable_irq(USARTx_IRQHandler, USARTx_IRQn, 0);

  gpio_peripheral(GPIO_Rx, GPIO_FUNCTION(7), 1);
  gpio_peripheral(GPIO_Tx, GPIO_FUNCTION(7), 0);
}
DECL_INIT(serial_init);

void serial1_init(void) {
  enable_pclock((uint32_t)USART1);

  uint32_t pclk = get_pclock_frequency((uint32_t)USART1);
  uint32_t div = DIV_ROUND_CLOSEST(pclk, 115200);
  USART1->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos) |
                 ((div % 16) << USART_BRR_DIV_Fraction_Pos));
  USART1->CR1 = CR1_FLAGS;
  // serial1_enable_tx_irq();
  armcm_enable_irq(USART1_IRQHandler, USART1_IRQn, 0);
  gpio_peripheral(GPIO('A', 10), GPIO_FUNCTION(7), 1);
  gpio_peripheral(GPIO('A', 9), GPIO_FUNCTION(7), 0);
}

/**
 * @description: 串口1初始化
 * @author: your name
 * @return {*}
 */
void usart1_init(void) {
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* 第1步： 配置GPIO */
  /* TX = PA9   RX = PA10 */
  /* 打开 GPIO 时钟 */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* 打开 UART 时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* 将 PA9 映射为 USART1_TX */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

  /* 将 PA10 映射为 USART1_RX */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  /* 配置 USART Tx 为复用功能 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* 输出类型为推挽 */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   /* 内部上拉电阻使能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;   /* 复用模式 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* 配置 USART Rx 为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* 第2步： 配置串口硬件参数 */
  USART_InitStructure.USART_BaudRate = 115200; /* 波特率 */
  USART_InitStructure.USART_WordLength =
      USART_WordLength_8b;  // USART_WordLength_9b
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;  //  USART_Parity_Even
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); /* 使能接收中断 */
  USART_Cmd(USART1, ENABLE);                     /* 使能串口 */

  /* 使能串口1中断 */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  armcm_enable_irq(USART1_IRQHandler, USART1_IRQn, 0);
}

static void UART1_Send(char data) {
  // 等待发送数据寄存器为空
  while (!(USART1->SR & USART_SR_TC))
    ;
  // 将数据写入发送数据寄存器
  USART1->DR = data;
}

static void UART1_SendString(char *str) {
  while (*str) {
    UART1_Send(*str++);
  }
}

void UART1_Printf(const char *fmt, ...) {
  char buffer[256];  // 输出缓冲区，你可能需要根据你的需求调整大小

  va_list args;
  va_start(args, fmt);
  vsprintf(buffer, fmt, args);
  va_end(args);
  UART1_SendString(buffer);
}

static int _write(int file, char *ptr, int len) {
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    UART1_Send(*ptr++);
  }
  return len;
}
