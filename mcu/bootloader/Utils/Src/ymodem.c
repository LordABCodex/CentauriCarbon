/*
*********************************************************************************************************
*
*	模块名称 : Ymodem协议
*	文件名称 : ymodem.c
*	版    本 : V1.0
*	说    明 : Ymodem协议
*
*	修改记录 :
*		版本号  日期         作者        说明
*		V1.0    2022-08-08  Eric2013     首发
*
*	Copyright (C), 2022-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "common.h"
#include "ymodem.h"
#include "string.h"
#include "bsp_cpu_flash.h"
#include "main.h"
#include "pfifo.h"
#include "usbd_cdc_if.h"
uint8_t FileName[1024] = {0};
uint16_t Cal_CRC16(const uint8_t *data, uint32_t size);

/*
*********************************************************************************************************
*	函 数 名: Receive_Byte
*	功能说明: 接收发送端发来的字符
*	形    参：c  字符
*             timeout  溢出时间
*	返 回 值: 0 接收成功， -1 接收失败
*********************************************************************************************************
*/
static int32_t Receive_Byte(uint8_t *c, uint32_t timeout)
{
	volatile uint32_t in_tick = HAL_GetTick();
	while (HAL_GetTick() - in_tick < timeout)
	{
#if OTA_INTERFACE == USE_USB
		if (cdc_pfifo_read_buf(c, 1) == 1)
#elif OTA_INTERFACE == USE_UART
		if (usart2_pfifo_read_buf(c, 1) == 1)
#endif
		{
			return 0;
		}
	}
	return -1;
}

/*
*********************************************************************************************************
*	函 数 名: Send_Byte
*	功能说明: 发送一个字节数据
*	形    参：c  字符
*	返 回 值: 0
*********************************************************************************************************
*/
static uint32_t Send_Byte(uint8_t c)
{
#if OTA_INTERFACE == USE_USB
	CDC_Transmit_FS(&c, 1);
#elif OTA_INTERFACE == USE_UART
	usart2_send_buf(&c, 1);
#endif
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: Receive_Packet
*	功能说明: 接收一包数据
*	形    参：data 数据
*             length 数据大小
*             timeout  0 传输结束
*                      -1 发送端终止传输
*                      >0 数据包长度
*	返 回 值: 0  正常返回
*             -1 时间溢出或数据包错误
*             1  用户终止
			  2  正常结束
*********************************************************************************************************
*/
static int32_t Receive_Packet(uint8_t *data, int32_t *length, uint32_t timeout)
{
	uint16_t i, packet_size;
	uint8_t c;
	uint16_t crc;

	*length = 0;

	/* 接收一个字符 */
	if (Receive_Byte(&c, timeout) != 0)
	{
		return -1;
	}
	switch (c)
	{
	/* SOH表示数据区有128字节 */
	case SOH:
		packet_size = PACKET_SIZE;
		break;

	/* STX表示数据区有1k字节 */
	case STX:
		packet_size = PACKET_1K_SIZE;
		break;

	/* 传输结束 end of transmission */
	case EOT:
		return 2;

	/* 连续的两个CA信号终止传输 */
	case CA:
		/* 收到两个连续的CA信号 */
		if ((Receive_Byte(&c, timeout) == 0) && (c == CA))
		{
			*length = -1;
			return 0;
		}
		/* 只收到一个CA信号 */
		else
		{
			return -1;
		}

	/* 用户终止传输 */
	case ABORT1:
	case ABORT2:
		return 1;

	default:
		return -1;
	}

	*data = c;
	for (i = 1; i < (packet_size + PACKET_OVERHEAD); i++) // 根据数据包大小接收数据直到收完。
	{
		if (Receive_Byte(data + i, timeout) != 0)
		{
			return -1;
		}
	}
	/* 第PACKET_SEQNO_COMP_INDEX（数字2）字节是PACKET_SEQNO_INDEX（数字1）字节的反码 */
	if (data[PACKET_SEQNO_INDEX] != ((data[PACKET_SEQNO_COMP_INDEX] ^ 0xff) & 0xff))
	{
		return -1;
	}

	/* 计算CRC */
	crc = data[packet_size + PACKET_HEADER] << 8;
	crc += data[packet_size + PACKET_HEADER + 1];
	if (Cal_CRC16(&data[PACKET_HEADER], packet_size) != crc)
	{
		printf("crc error\r\n");
		return -1;
	}

	/* 数据区长度 */
	*length = packet_size;
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: Receive_Packet
*	功能说明: 按照ymodem协议接收数据
*	形    参: buf 数据首地址
*	返 回 值: 文件大小
*********************************************************************************************************
*/
int32_t Ymodem_Receive(uint32_t appadr, machine_para_t *machine_para)
{
	uint32_t TotalSize = 0;
	uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD], file_size[FILE_SIZE_LENGTH], *file_ptr;
	int32_t i, packet_length, session_done, file_done, packets_received, errors, session_begin, size = 0;
	uint32_t flashdestination, ramsource;
	uint8_t buf_temp[2048] = {0};
	uint8_t *buf_ptr = NULL;
	/* 初始化flash编程首地址 */
	flashdestination = appadr;

	/* 接收数据并进行flash编程 */
	for (session_done = 0, errors = 0, session_begin = 0;;)
	{
		for (packets_received = 0, file_done = 0, buf_ptr = buf_temp;;)
		{
			switch (Receive_Packet(packet_data, &packet_length, NAK_TIMEOUT)) // 数据预处理
			{
			/* 返回0表示接收成功 */
			case 0:
				errors = 0;
				switch (packet_length)
				{
				/* 发送端终止传输 */
				case -1:
					Send_Byte(ACK);
					return 0;

				/* 传输结束 */
				case 0:
					Send_Byte(ACK);
					file_done = 1;
					break;

				/* 接收数据 */
				default:
					if ((packet_data[PACKET_SEQNO_INDEX] & 0xff) != (packets_received & 0xff)) // 数据包序号与本地记录不同,出错。
					{
						printf("seq error\r\n");
						Send_Byte(NAK);
					}
					else
					{
						if (packets_received == 0) // 第一包数据
						{
							/* 文件名数据包 */
							if (packet_data[PACKET_HEADER] != 0)
							{
								/* 读取文件名 */
								for (i = 0, file_ptr = packet_data + PACKET_HEADER; (*file_ptr != 0) && (i < FILE_NAME_LENGTH);)
								{
									FileName[i++] = *file_ptr++;
								}
								/* 文件名末尾加结束符 */
								FileName[i++] = '\0';

								/* 读取文件大小 */
								for (i = 0, file_ptr++; (*file_ptr != ' ') && (i < FILE_SIZE_LENGTH);)
								{
									file_size[i++] = *file_ptr++;
								}
								file_size[i++] = '\0';

								printf("FileName:%s\r\n", FileName);
								printf("FileSize:%s\r\n", file_size);

								/* 将文件大小的字符串转换成整型数据 */
								Str2Int(file_size, &size);

								/* 检测文件大小是否比flash空间大 */
								if (size > (FLASH_SIZE + 1))
								{
									/* 终止传输 */
									Send_Byte(CA);
									Send_Byte(CA);
									return -1;
								}

								Send_Byte(ACK);
								Send_Byte(CRC16);
							}
							/* 文件名数据包处理完，终止此部分，开始接收数据 */
							else
							{
								Send_Byte(ACK);
								file_done = 1;
								session_done = 1;
								break;
							}
						}

						/* 数据包 */
						else
						{
							/* 读取数据 */
							if (packets_received == 1)
							{
								machine_para_t *recv_machine_para = NULL;
								recv_machine_para = (machine_para_t *)(packet_data + PACKET_HEADER);
								if (machine_para->extra[0] == 1 && recv_machine_para->major_version == machine_para->major_version && recv_machine_para->minor_version == machine_para->minor_version && recv_machine_para->patch_version == machine_para->patch_version)
//								if (0)
								{
									/* 终止传输 */
									printf("send same version\r\n");
									Send_Byte(CA);
									Send_Byte(CA);
									return -3;
								}
								else
								{
									printf("send different version start upgrade\r\n");
									printf("send major_version : %d\n", recv_machine_para->major_version);
									printf("send minor_version : %d\n", recv_machine_para->minor_version);
									printf("send patch_version : %d\n", recv_machine_para->patch_version);
									recv_machine_para->extra[0] = 0; // 升级flag，置零，启动应用再置1
									/* 擦除用户区flash */
									bsp_EraseCpuFlash(ADDR_FLASH_SECTOR_3);
									bsp_EraseCpuFlash(ADDR_FLASH_SECTOR_4);
									bsp_EraseCpuFlash(ADDR_FLASH_SECTOR_5);
								}
							}
							memcpy(buf_ptr, packet_data + PACKET_HEADER, packet_length);
							ramsource = (uint32_t)buf_ptr;
							/* 扇区编程 */
							uint8_t ucState = bsp_WriteCpuFlash((uint32_t)(flashdestination + TotalSize), (uint8_t *)ramsource, (TotalSize + packet_length) > size ? size - TotalSize : packet_length);
							TotalSize = (TotalSize + packet_length) > size ? size : (TotalSize + packet_length);
							printf("progress:%d\r\n", (TotalSize * 100) / size);
							/* 如果返回非0，表示编程失败 */
							if (ucState != 0)
							{
								/* 终止传输 */
								Send_Byte(CA);
								Send_Byte(CA);
								return -5;
							}

							Send_Byte(ACK);
						}
						/* 接收数据包递增 */
						packets_received++;
						session_begin = 1;
					}
				}
				break;

			/* 用户终止传输 */
			case 1:
				Send_Byte(CA);
				Send_Byte(CA);
				return -3;
			case 2:
				Send_Byte(NAK);
				if (Receive_Byte(&packet_data[0], NAK_TIMEOUT) == 0)
				{
					if (packet_data[0] == EOT) // 只支持一次传输
					{
						Send_Byte(ACK);
						// Send_Byte(CRC16);
						file_done = 1;
						session_done = 1;
					}
					else
					{
						Send_Byte(CA);
						Send_Byte(CA);
						return -4;
					}
				}
				break;
			/* 其它 */
			default:
				if (session_begin > 0)
				{
					errors++;
				}

				if (errors > MAX_ERRORS)
				{
					Send_Byte(CA);
					Send_Byte(CA);
					return 0;
				}

				Send_Byte(CRC16);
				break;
			}

			if (file_done != 0)
			{
				break;
			}
		}

		if (session_done != 0)
		{
			break;
		}
	}

	return (int32_t)TotalSize;
}

/*
*********************************************************************************************************
*	函 数 名: Ymodem_CheckResponse
*	功能说明: 响应
*	形    参: c 字符
*	返 回 值: 0
*********************************************************************************************************
*/
int32_t Ymodem_CheckResponse(uint8_t c)
{
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: Ymodem_PrepareIntialPacket
*	功能说明: 准备第一包要发送的数据
*	形    参: data 数据
*             fileName 文件名
*             length   文件大小
*	返 回 值: 0
*********************************************************************************************************
*/
void Ymodem_PrepareIntialPacket(uint8_t *data, const uint8_t *fileName, uint32_t *length)
{
	uint16_t i, j;
	uint8_t file_ptr[10];

	/* 第一包数据的前三个字符  */
	data[0] = SOH; /* soh表示数据包是128字节 */
	data[1] = 0x00;
	data[2] = 0xff;

	/* 文件名 */
	for (i = 0; (fileName[i] != '\0') && (i < FILE_NAME_LENGTH); i++)
	{
		data[i + PACKET_HEADER] = fileName[i];
	}

	data[i + PACKET_HEADER] = 0x00;

	/* 文件大小转换成字符 */
	Int2Str(file_ptr, *length);
	for (j = 0, i = i + PACKET_HEADER + 1; file_ptr[j] != '\0';)
	{
		data[i++] = file_ptr[j++];
	}

	/* 其余补0 */
	for (j = i; j < PACKET_SIZE + PACKET_HEADER; j++)
	{
		data[j] = 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: UpdateCRC16
*	功能说明: 上次计算的CRC结果 crcIn 再加上一个字节数据计算CRC
*	形    参: crcIn 上一次CRC计算结果
*             byte  新添加字节
*	返 回 值: 无
*********************************************************************************************************
*/
uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte)
{
	uint32_t crc = crcIn;
	uint32_t in = byte | 0x100;

	do
	{
		crc <<= 1;
		in <<= 1;
		if (in & 0x100)
			++crc;
		if (crc & 0x10000)
			crc ^= 0x1021;
	} while (!(in & 0x10000));

	return crc & 0xffffu;
}

/*
*********************************************************************************************************
*	函 数 名: Cal_CRC16
*	功能说明: 计算一串数据的CRC
*	形    参: data  数据
*             size  数据长度
*	返 回 值: CRC计算结果
*********************************************************************************************************
*/
uint16_t Cal_CRC16(const uint8_t *data, uint32_t size)
{
	uint32_t crc = 0;
	const uint8_t *dataEnd = data + size;

	while (data < dataEnd)
		crc = UpdateCRC16(crc, *data++);

	crc = UpdateCRC16(crc, 0);
	crc = UpdateCRC16(crc, 0);

	return crc & 0xffffu;
}
