// Code to setup clocks on stm32f2/stm32f4
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"          // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h"  // VectorTable
#include "board/armcm_reset.h" // try_request_canboot
#include "board/serial.h"
#include "board/irq.h"         // irq_disable
#include "board/misc.h"        // bootloader_request
#include "command.h"           // DECL_CONSTANT_STR
#include "internal.h"          // enable_pclock
#include "sched.h"             // sched_main
#include <stdarg.h>
#include <stdio.h>
#include "board/serial_irq.h" // serial_rx_byte
#include <stdio.h>            // 减少编译器的警告#pragma GCC diagnostic ignored "-Wunknown-pragmas"//确保程序中不包含任何半主机的函数
#include "cpu_flash_operate.h"
#include "rtc.h"
#include "cli.h"
#include "hx717_exit.h"
/****************************************************************
 * Clock setup
 ****************************************************************/

#define FREQ_PERIPH_DIV (CONFIG_MACH_STM32F401 ? 2 : 4)
#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / FREQ_PERIPH_DIV)
#define FREQ_USB 48000000
#define PARA_AREA_ADDR ADDR_FLASH_SECTOR_2
typedef struct
{
    uint8_t magic[4];
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t patch_version;
    uint8_t board_type;
    uint8_t extra[4]; // extra[0] is_upgraded
    uint32_t file_len;
    uint8_t md5[16];
} machine_para_t;
machine_para_t machine_para = {0};
// Map a peripheral address to its enable bits
struct cline
lookup_clock_line(uint32_t periph_base)
{
    if (periph_base >= AHB1PERIPH_BASE)
    {
        uint32_t bit = 1 << ((periph_base - AHB1PERIPH_BASE) / 0x400);
        return (struct cline){.en = &RCC->AHB1ENR, .rst = &RCC->AHB1RSTR, .bit = bit};
    }
    else if (periph_base >= APB2PERIPH_BASE)
    {
        uint32_t bit = 1 << ((periph_base - APB2PERIPH_BASE) / 0x400);
        if (bit & 0x700)
            // Skip ADC peripheral reset as they share a bit
            return (struct cline){.en = &RCC->APB2ENR, .bit = bit};
        return (struct cline){.en = &RCC->APB2ENR, .rst = &RCC->APB2RSTR, .bit = bit};
    }
    else
    {
        uint32_t bit = 1 << ((periph_base - APB1PERIPH_BASE) / 0x400);
        return (struct cline){.en = &RCC->APB1ENR, .rst = &RCC->APB1RSTR, .bit = bit};
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - AHB1PERIPH_BASE) / 0x400;
    RCC->AHB1ENR |= 1 << rcc_pos;
    RCC->AHB1ENR;
}

#if !CONFIG_STM32_CLOCK_REF_INTERNAL
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PH0,PH1");
#endif

// Clock configuration
static void
enable_clock_stm32f20x(void)
{
#if CONFIG_MACH_STM32F207
    uint32_t pll_base = 1000000, pll_freq = CONFIG_CLOCK_FREQ * 2, pllcfgr;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL)
    {
        // Configure 120Mhz PLL from external crystal (HSE)
        uint32_t div = CONFIG_CLOCK_REF_FREQ / pll_base;
        RCC->CR |= RCC_CR_HSEON;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
    }
    else
    {
        // Configure 120Mhz PLL from internal 16Mhz oscillator (HSI)
        uint32_t div = 16000000 / pll_base;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
    }
    RCC->PLLCFGR = (pllcfgr | ((pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | ((pll_freq / FREQ_USB) << RCC_PLLCFGR_PLLQ_Pos));
    RCC->CR |= RCC_CR_PLLON;
#endif
}

static void
enable_clock_stm32f40x(void)
{
#if CONFIG_MACH_STM32F401 || CONFIG_MACH_STM32F4x5
    uint32_t pll_base = (CONFIG_STM32_CLOCK_REF_25M) ? 1000000 : 2000000;
    uint32_t pllp = (CONFIG_MACH_STM32F401) ? 4 : 2;
    uint32_t pll_freq = CONFIG_CLOCK_FREQ * pllp, pllcfgr;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL)
    {
        // Configure 168Mhz PLL from external crystal (HSE)
        uint32_t div = CONFIG_CLOCK_REF_FREQ / pll_base;
        RCC->CR |= RCC_CR_HSEON;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
    }
    else
    {
        // Configure 168Mhz PLL from internal 16Mhz oscillator (HSI)
        uint32_t div = 16000000 / pll_base;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
    }
    RCC->PLLCFGR = (pllcfgr | ((pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos) | (((pllp >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos) | ((pll_freq / FREQ_USB) << RCC_PLLCFGR_PLLQ_Pos));
    RCC->CR |= RCC_CR_PLLON;
#endif
}

static void
enable_clock_stm32f446(void)
{
#if CONFIG_MACH_STM32F446
    uint32_t pll_base = 2000000, pll_freq = CONFIG_CLOCK_FREQ * 2, pllcfgr;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL)
    {
        // Configure 180Mhz PLL from external crystal (HSE)
        uint32_t div = CONFIG_CLOCK_REF_FREQ / pll_base;
        RCC->CR |= RCC_CR_HSEON;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
    }
    else
    {
        // Configure 180Mhz PLL from internal 16Mhz oscillator (HSI)
        uint32_t div = 16000000 / pll_base;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
    }
    RCC->PLLCFGR = (pllcfgr | ((pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | ((pll_freq / FREQ_USB) << RCC_PLLCFGR_PLLQ_Pos) | (6 << RCC_PLLCFGR_PLLR_Pos));
    RCC->CR |= RCC_CR_PLLON;

    // Enable "over drive"
    enable_pclock(PWR_BASE);
    PWR->CR = (3 << PWR_CR_VOS_Pos) | PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY))
        ;
    PWR->CR = (3 << PWR_CR_VOS_Pos) | PWR_CR_ODEN | PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY))
        ;

    // Enable 48Mhz USB clock for USB or for SDIO
    if (CONFIG_USB || CONFIG_HAVE_GPIO_SDIO)
    {
        uint32_t ref = (CONFIG_STM32_CLOCK_REF_INTERNAL
                            ? 16000000
                            : CONFIG_CLOCK_REF_FREQ);
        uint32_t plls_base = 2000000, plls_freq = FREQ_USB * 4;
        RCC->PLLSAICFGR = (((ref / plls_base) << RCC_PLLSAICFGR_PLLSAIM_Pos) | ((plls_freq / plls_base) << RCC_PLLSAICFGR_PLLSAIN_Pos) | (((plls_freq / FREQ_USB) / 2 - 1) << RCC_PLLSAICFGR_PLLSAIP_Pos) | ((plls_freq / FREQ_USB) << RCC_PLLSAICFGR_PLLSAIQ_Pos));
        RCC->CR |= RCC_CR_PLLSAION;
        while (!(RCC->CR & RCC_CR_PLLSAIRDY))
            ;

        RCC->DCKCFGR2 = RCC_DCKCFGR2_CK48MSEL;
    }
    else
    {
        // Reset value just in case the booloader modified the default value
        RCC->DCKCFGR2 = 0;
    }

    // Set SDIO clk to PLL48CLK
    if (CONFIG_HAVE_GPIO_SDIO)
    {
        MODIFY_REG(RCC->DCKCFGR2, RCC_DCKCFGR2_SDIOSEL, 0);
    }
#endif
}

// Main clock setup called at chip startup
static void
clock_setup(void)
{
    // Configure and enable PLL
    if (CONFIG_MACH_STM32F207)
        enable_clock_stm32f20x();
    else if (CONFIG_MACH_STM32F401 || CONFIG_MACH_STM32F4x5)
        enable_clock_stm32f40x();
    else
        enable_clock_stm32f446();

    // Set flash latency
    FLASH->ACR = (FLASH_ACR_LATENCY_5WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);

    // Wait for PLL lock
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Switch system clock to PLL
    if (FREQ_PERIPH_DIV == 2)
        RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_SW_PLL;
    else
        RCC->CFGR = RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV4 | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
        ;
}

/****************************************************************
 * Bootloader
 ****************************************************************/

// Reboot into USB "HID" bootloader
static void
usb_hid_bootloader(void)
{
    irq_disable();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB1ENR;
    PWR->CR |= PWR_CR_DBP;
    RTC->BKP4R = 0x424C; // HID Bootloader magic key
    PWR->CR &= ~PWR_CR_DBP;
    NVIC_SystemReset();
}

// Handle reboot requests
void bootloader_request(void)
{
    try_request_canboot();
    if (CONFIG_STM32_FLASH_START_4000)
        usb_hid_bootloader();
    dfu_reboot();
}


/****************************************************************
 * Startup
 ****************************************************************/

// Main entry point - called from armcm_boot.c:ResetHandler()
void armcm_main(void)
{

#if 0
    Stm32_Clock_Init(84,12,2,7);
    // my_NVIC_set_vectortable((uint32_t)VectorTable, 0xC000);
    SCB->VTOR = (uint32_t)VectorTable;
    
    usart1_init();
    Test();
#else 
    dfu_reboot_check();

    // Run SystemInit() and then restore VTOR
    // SystemInit();
    Stm32_Clock_Init(84,12,2,7);
    // Stm32_Clock_Init(336,16,4,7);
    SCB->VTOR = (uint32_t)VectorTable;

    // Reset peripheral clocks (for some bootloaders that don't)
    RCC->AHB1ENR = 0x38000;
    RCC->AHB2ENR = 0;
    RCC->APB1ENR = 0;
    RCC->APB2ENR = 0;


    clock_setup();
    // serial1_init();
    usart1_init();
    cpu_flash_read(PARA_AREA_ADDR, (uint8_t *)&machine_para, sizeof(machine_para_t));
     
    UART1_Printf("\r\n");
    UART1_Printf("machine_para.major_version = %d\r\n", machine_para.major_version);
    UART1_Printf("machine_para.minor_version = %d\r\n", machine_para.minor_version);
    UART1_Printf("machine_para.patch_version = %d\r\n", machine_para.patch_version);
    UART1_Printf("machine_para.is_upgraded = %d\r\n", machine_para.extra[0]); 
    if (machine_para.extra[0] == 0)
    {
        machine_para.extra[0] = 1;
        cpu_write_flash_from_addr(PARA_AREA_ADDR, &machine_para, sizeof(machine_para_t));
    }

    struct TimeStructure time;
    time.year = 2024;
    time.mon = 4;
    time.date = 26;
    time.hour = 16;
    time.min = 25;
    time.sec = 0;
    time.week = 5;
    rtc_settime(&time);
    
    cli_init();

    UART1_Printf("system init\r\n");

    // struct TimeStructure ptime;
    // rtc_gettime(&ptime);
    // UART1_Printf("%d/%d/%d %d:%d:%d\r\n", ptime.year, ptime.mon, ptime.date,
    //     ptime.hour, ptime.min, ptime.sec);

    sched_wake_tasks();
    sched_main();
#endif
}
