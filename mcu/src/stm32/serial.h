#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>  // uint32_t

extern void serial1_init(void);
extern void usart1_init(void);
extern void UART1_Printf(const char *fmt, ...);
extern int usart1_pend(char *pchar);
extern int usart1_post(char temp);

#endif  // serial.h
