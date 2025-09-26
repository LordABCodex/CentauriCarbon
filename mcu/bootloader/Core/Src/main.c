/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "pfifo.h"
#include "bsp_cpu_flash.h"
#include "ymodem.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_START_ADDRESS ADDR_FLASH_SECTOR_3
#define PARA_AREA_ADDR ADDR_FLASH_SECTOR_2
#define ENABLE_INT() __set_PRIMASK(0)
#define DISABLE_INT() __set_PRIMASK(1)
#define USART2_FIFO_SIZE 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
machine_para_t machine_para = {0};
#if OTA_INTERFACE == USE_UART
static fifo_rx_def_t usart2_recv_fifo = {.buffer = NULL, .in = 0, .out = 0, .size = 0, .error = 0, .last_cnt = 0};
static uint8_t usart2_dma_buff[USART2_FIFO_SIZE] = {0};
static uint8_t usart2_fifo_buff[USART2_FIFO_SIZE] = {0};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void jump_to_address(uint32_t app_address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#if OTA_INTERFACE == USE_USB
  MX_USB_DEVICE_Init();
#elif OTA_INTERFACE == USE_UART
  MX_DMA_Init();
  MX_USART2_UART_Init();
#endif
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (bsp_ReadCpuFlash(PARA_AREA_ADDR, (uint8_t *)&machine_para, sizeof(machine_para_t)) == 0)
  {
    printf("magic : ");
    for (uint32_t i = 0; i < 4; i++)
    {
      printf("%02x ", machine_para.magic[i]);
    }
    printf("\r\n");
    printf("major_version : %d\r\n", machine_para.major_version);
    printf("minor_version : %d\r\n", machine_para.minor_version);
    printf("patch_version : %d\r\n", machine_para.patch_version);
    printf("board_type : %d\r\n", machine_para.board_type);
    printf("is_upgraded : %d\r\n", machine_para.extra[0]);
    printf("file_len : %d\r\n", machine_para.file_len);
    printf("md5 : ");
    for (uint32_t i = 0; i < 16; i++)
    {
      printf("%02x ", machine_para.md5[i]);
    }
    printf("\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    printf("### ymodem : %d ###\r\n", Ymodem_Receive(PARA_AREA_ADDR, &machine_para));
#if OTA_INTERFACE == USE_USB
    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);
    
#elif OTA_INTERFACE == USE_UART
    while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX_RX)
    {
      HAL_Delay(1);
    }
    HAL_UART_DeInit(&huart2);
#endif
    jump_to_address(APP_START_ADDRESS);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
#if OTA_INTERFACE == USE_UART
  pfifo_init(&usart2_recv_fifo, usart2_fifo_buff, sizeof(usart2_fifo_buff));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_dma_buff, sizeof(usart2_dma_buff));
#endif
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f)
{
  USART1->DR = ch;

  while ((USART1->SR & USART_SR_TC) == 0)
    ;

  return ch;
}

static void jump_to_address(uint32_t app_address)
{
  uint32_t i = 0;
  void (*AppJump)(void);

  DISABLE_INT();

  HAL_RCC_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  for (i = 0; i < 8; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  ENABLE_INT();

  AppJump = (void (*)(void))(*((uint32_t *)(app_address + 4)));

  __set_MSP(*(uint32_t *)app_address);

  __set_CONTROL(0);

  AppJump();
  while (1)
  {
    printf("Jump To APP %d Fail\n", app_address);
  }
}
#if OTA_INTERFACE == USE_UART
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == USART2)
  {
    // printf("usart2 recv %u\n", size);
    // for (uint32_t i = 0; i < size; i++)
    // {
    //   printf("%02x ", usart2_dma_buff[i]);
    // }
    // printf("\n");
    pfifo_write_buff(&usart2_recv_fifo, usart2_dma_buff, size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_dma_buff, sizeof(usart2_dma_buff));
  }
}
uint32_t usart2_pfifo_read_buf(uint8_t *buffer, uint32_t len)
{
  return (pfifo_read_buff(&usart2_recv_fifo, buffer, len));
}
HAL_StatusTypeDef usart2_send_buf(uint8_t *buffer, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    while ((USART2->SR & 0X40) == 0)
      ;
    USART2->DR = buffer[i];
    // printf("usart2 send %02x\n", buffer[0]);
  }
  return HAL_OK;
  // return HAL_UART_Transmit(&huart2, buffer, len, 1000);
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
