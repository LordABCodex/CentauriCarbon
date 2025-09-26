#include "internal.h"
#include "cpu_flash_operate.h"
#include "stm32f401xc.h"
#include "stm32f4xx.h"
#include "board/misc.h"
#define FLASH_TIMEOUT_VALUE 5000000U /* 5 s */
#if CONFIG_MACH_STM32F401
/*
*********************************************************************************************************
*	函 数 名: cpu_flash_wait_busy
*	功能说明: 等待 flash 空闲
*	形    参:  buf : timeout
*	返 回 值: 0=成功，-1=失败
*********************************************************************************************************
*/
static int cpu_flash_wait_busy(uint32_t timeout)
{
    uint32_t end = timer_read_time() + timer_from_us(timeout);
    while (READ_BIT(FLASH->SR, FLASH_SR_BSY) != 0)
    {
        if (!timer_is_before(timer_read_time(), end))
        {
            return -1;
        }
    }
    if (READ_BIT(FLASH->SR, FLASH_SR_EOP) != 0)
    {
        WRITE_REG(FLASH->SR, FLASH_SR_EOP);
    }
    if (READ_BIT(FLASH->SR, (FLASH_SR_SOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                             FLASH_SR_PGPERR | FLASH_SR_PGSERR)) != RESET)
    {
        return -1;
    }
    return 0;
}
/*
*********************************************************************************************************
*	函 数 名: cpu_flash_flush_caches
*	功能说明:
*	形    参:
*	返 回 值:
*********************************************************************************************************
*/
static void cpu_flash_flush_caches(void)
{
    /* Flush instruction cache  */
    if (READ_BIT(FLASH->ACR, FLASH_ACR_ICEN) != 0)
    {
        /* Disable instruction cache  */
        FLASH->ACR &= (~FLASH_ACR_ICEN);
        /* Reset instruction cache */
        FLASH->ACR |= FLASH_ACR_ICRST;
        FLASH->ACR &= ~FLASH_ACR_ICRST;
        /* Enable instruction cache */
        FLASH->ACR |= FLASH_ACR_ICEN;
    }

    /* Flush data cache */
    if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != 0)
    {
        /* Disable data cache  */
        FLASH->ACR &= (~FLASH_ACR_DCEN);
        /* Reset data cache */
        FLASH->ACR |= FLASH_ACR_DCRST;
        FLASH->ACR &= ~FLASH_ACR_DCRST;
        /* Enable data cache */
        FLASH->ACR |= FLASH_ACR_DCEN;
    }
}
static int cpu_flash_unlock(void)
{
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0)
    {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);

        /* Verify Flash is unlocked */
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0)
        {
            return -1;
        }
    }
    return 0;
}
static int cpu_flash_lock(void)
{
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);
    return 0;
}
/*
*********************************************************************************************************
*	函 数 名: cpu_flash_erase_sector
*	功能说明: 擦除flash扇区
*	形    参:  sector_index : 目标sector编号（Sector number）
*	返 回 值: 0=成功，-1=失败
*********************************************************************************************************
*/
static int cpu_flash_erase_sector(uint32_t sector_index)
{
    if (!IS_FLASH_SECTOR(sector_index))
    {
        return -1;
    }
    if (cpu_flash_wait_busy(FLASH_TIMEOUT_VALUE) == -1)
        return -1;
    /* If the previous operation is completed, proceed to erase the sector */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_PSIZE_WORD;
    CLEAR_BIT(FLASH->CR, FLASH_CR_SNB);
    FLASH->CR |= FLASH_CR_SER | (sector_index << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    if (cpu_flash_wait_busy(FLASH_TIMEOUT_VALUE) == -1)
        return -1;
    cpu_flash_flush_caches();
    return 0;
}
/*
*********************************************************************************************************
*	函 数 名: cpu_flash_get_sector_from_addr
*	功能说明: 根据地址计算扇区首地址
*	形    参:  无
*	返 回 值: 扇区首地址
*********************************************************************************************************
*/
static uint32_t cpu_flash_get_sector_from_addr(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }

    return sector;
}
/*
*********************************************************************************************************
*	函 数 名: cpu_flash_cmp
*	功能说明: 比较Flash指定地址的数据.
*	形    参: flash_addr : Flash地址
*			 buf : 数据缓冲区
*			 size : 数据大小（单位是字节）
*	返 回 值:
*			FLASH_IS_EQU		0   Flash内容和待写入的数据相等，不需要擦除和写操作
*			FLASH_REQ_WRITE		1	Flash不需要擦除，直接写
*			FLASH_REQ_ERASE		2	Flash需要先擦除,再写
*			FLASH_PARAM_ERR		3	函数参数错误
*********************************************************************************************************
*/
static uint8_t cpu_flash_cmp(uint32_t flash_addr, uint8_t *buf, uint32_t size)
{
    uint32_t i;
    uint8_t ucIsEqu; /* 相等标志 */
    uint8_t ucByte;

    /* 如果偏移地址超过芯片容量，则不改写输出缓冲区 */
    if (flash_addr + size > FLASH_BASE_ADDR + FLASH_SIZE)
    {
        return FLASH_PARAM_ERR; /*　函数参数错误　*/
    }

    /* 长度为0时返回正确 */
    if (size == 0)
    {
        return FLASH_IS_EQU; /* Flash内容和待写入的数据相等 */
    }

    ucIsEqu = 1; /* 先假设所有字节和待写入的数据相等，如果遇到任何一个不相等，则设置为 0 */
    for (i = 0; i < size; i++)
    {
        ucByte = *(uint8_t *)flash_addr;

        if (ucByte != *buf)
        {
            if (ucByte != 0xFF)
            {
                return FLASH_REQ_ERASE; /* 需要擦除后再写 */
            }
            else
            {
                ucIsEqu = 0; /* 不相等，需要写 */
            }
        }

        flash_addr++;
        buf++;
    }

    if (ucIsEqu == 1)
    {
        return FLASH_IS_EQU; /* Flash内容和待写入的数据相等，不需要擦除和写操作 */
    }
    else
    {
        return FLASH_REQ_WRITE; /* Flash不需要擦除，直接写 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: cpu_flash_read
*	功能说明: 读取CPU Flash的内容
*	形    参:  buf : 目标缓冲区
*			 flash_addr : 起始地址
*			 Size : 数据大小（单位是字节）
*	返 回 值: 0=成功，1=失败
*********************************************************************************************************
*/
uint8_t cpu_flash_read(uint32_t flash_addr, uint8_t *buf, uint32_t Size)
{
    uint32_t i;

    /* 如果偏移地址超过芯片容量，则不改写输出缓冲区 */
    if (flash_addr + Size > FLASH_BASE_ADDR + FLASH_SIZE)
    {
        return 1;
    }

    /* 长度为0时不继续操作,否则起始地址为奇地址会出错 */
    if (Size == 0)
    {
        return 1;
    }

    for (i = 0; i < Size; i++)
    {
        *buf++ = *(uint8_t *)flash_addr++;
    }

    return 0;
}
/*
*********************************************************************************************************
*	函 数 名: cpu_flash_erase_sector_from_addr
*	功能说明: 擦除CPU FLASH一个扇区
*	形    参: _ulFlashAddr : Flash地址
*	返 回 值: 0 成功， -1 失败
*
*********************************************************************************************************
*/
int cpu_flash_erase_sector_from_addr(uint32_t flash_addr)
{
    int ret = 0;
    cpu_flash_unlock();
    uint32_t sector_number = cpu_flash_get_sector_from_addr(flash_addr);
    ret = cpu_flash_erase_sector(sector_number);
    cpu_flash_lock();
    return ret;
}

/*
*********************************************************************************************************
*	函 数 名: cpu_write_flash_from_addr
*	功能说明: 写数据到CPU 内部Flash。
*	形    参: flash_addr : Flash地址
*			 buf : 数据缓冲区
*			 size : 数据大小（单位是字节）
*	返 回 值: 0-成功，1-数据长度或地址溢出，2-写Flash出错(估计Flash寿命到)
*********************************************************************************************************
*/
uint8_t cpu_write_flash_from_addr(uint32_t flash_addr, uint8_t *buf, uint32_t size)
{
    uint32_t i;
    uint8_t ret;

    /* 如果偏移地址超过芯片容量，则不改写输出缓冲区 */
    if (flash_addr + size > FLASH_BASE_ADDR + FLASH_SIZE)
    {
        return 1;
    }

    /* 长度为0时不继续操作  */
    if (size == 0)
    {
        return 0;
    }

    ret = cpu_flash_cmp(flash_addr, buf, size);

    if (ret == FLASH_IS_EQU)
    {
        return 0;
    }

    __set_PRIMASK(1); /* 关中断 */

    /* 需要擦除 */
    if (ret == FLASH_REQ_ERASE)
    {
        cpu_flash_erase_sector_from_addr(flash_addr);
    }

    /* FLASH 解锁 */
    cpu_flash_unlock();

    /* 按字节模式编程（为提高效率，可以按字编程，一次写入4字节） */
    for (i = 0; i < size; i++)
    {
        if (cpu_flash_wait_busy(FLASH_TIMEOUT_VALUE) == 0)
        {
            /* If the previous operation is completed, proceed to program the new data */
            CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
            FLASH->CR |= FLASH_PSIZE_BYTE;
            FLASH->CR |= FLASH_CR_PG;
            *(__IO uint8_t *)flash_addr++ = *buf++;
            cpu_flash_wait_busy(FLASH_TIMEOUT_VALUE);
            FLASH->CR &= (~FLASH_CR_PG);
        }
    }
    /* Flash 加锁，禁止写Flash控制寄存器 */
    cpu_flash_lock();

    __set_PRIMASK(0); /* 开中断 */

    return 0;
}
#endif