
/*
*********************************************************************************************************
*
*	ģ������ : �����ļ�
*	�ļ����� : common.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	�޸ļ�¼ :
*		�汾��  ����         ����        ˵��
*		V1.0    2022-08-08  Eric2013     �׷�         
*
*	Copyright (C), 2022-2030, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __COMMON_H
#define __COMMON_H

#include "stdint.h"

#define IS_AF(c)  ((c >= 'A') && (c <= 'F'))
#define IS_af(c)  ((c >= 'a') && (c <= 'f'))
#define IS_09(c)  ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)  IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)  IS_09(c)
#define CONVERTDEC(c)  (c - '0')

#define CONVERTHEX_alpha(c)  (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)   (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))

#define SerialPutString(x) Serial_PutString((uint8_t*)(x))

/* ���ⲿ�ļ����õĺ��� */
void Int2Str(uint8_t* str,int32_t intnum);
uint32_t Str2Int(uint8_t *inputstr,int32_t *intnum);
#endif  

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
