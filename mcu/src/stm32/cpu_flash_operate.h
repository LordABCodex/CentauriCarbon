#ifndef __CPU_FLASH_OPERATE_H
#define __CPU_FLASH_OPERATE_H
#include <stdint.h> // uint...
#define FLASH_BASE_ADDR	0x08000000			/* Flash基地址 */
#define	FLASH_SIZE		(256*1024)		/* Flash 容量 */

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */


#define FLASH_IS_EQU		0   /* Flash内容和待写入的数据相等，不需要擦除和写操作 */
#define FLASH_REQ_WRITE		1	/* Flash不需要擦除，直接写 */
#define FLASH_REQ_ERASE		2	/* Flash需要先擦除,再写 */
#define FLASH_PARAM_ERR		3	/* 函数参数错误 */

#define FLASH_KEY1               0x45670123U
#define FLASH_KEY2               0xCDEF89ABU

uint8_t cpu_flash_read(uint32_t flash_addr, uint8_t *buf, uint32_t Size);
uint8_t cpu_write_flash_from_addr(uint32_t flash_addr, uint8_t *buf, uint32_t size);
int cpu_flash_erase_sector_from_addr(uint32_t flash_addr);
#define FLASH_PSIZE_BYTE           0x00000000U
#define FLASH_PSIZE_HALF_WORD      0x00000100U
#define FLASH_PSIZE_WORD           0x00000200U
#define FLASH_PSIZE_DOUBLE_WORD    0x00000300U
#define CR_PSIZE_MASK              0xFFFFFCFFU

#define FLASH_SECTOR_0     0U /*!< Sector Number 0   */
#define FLASH_SECTOR_1     1U /*!< Sector Number 1   */
#define FLASH_SECTOR_2     2U /*!< Sector Number 2   */
#define FLASH_SECTOR_3     3U /*!< Sector Number 3   */
#define FLASH_SECTOR_4     4U /*!< Sector Number 4   */
#define FLASH_SECTOR_5     5U /*!< Sector Number 5   */

#define IS_FLASH_SECTOR(SECTOR) (((SECTOR) == FLASH_SECTOR_0)   || ((SECTOR) == FLASH_SECTOR_1)   ||\
                                 ((SECTOR) == FLASH_SECTOR_2)   || ((SECTOR) == FLASH_SECTOR_3)   ||\
                                 ((SECTOR) == FLASH_SECTOR_4)   || ((SECTOR) == FLASH_SECTOR_5))
#endif // __CPU_FLASH_OPERATE_H