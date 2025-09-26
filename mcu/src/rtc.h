#ifndef __RTC_H__
#define __RTC_H__

#include <math.h>

// #include "stm32f4xx_pwr.h"
#include <stddef.h>

#include "board/serial.h"
#include "board/stm32f4xx_pwr.h"
#include "board/stm32f4xx_rcc.h"
#include "board/stm32f4xx_rtc.h"
#include "cli.h"

/* stm32f407的rtc年份是一个字节 */
#define YEAR_ALIGN (1970)
#define YEAR_MAX (2050)

#define ASSERT(a, b) \
  if (a) {           \
    return b;        \
  }

/* 日期格式 */
enum en_date_type {
  DATE_UTC = 0,
  DATE_BEIJING,
};

//时间结构体
struct TimeStructure {
  int sec;  /* 秒，正常范围0-59， 但允许至61*/
  int min;  /* 分钟，0-59                   */
  int hour; /* 小时， 0-23                  */
  int date; /* 日，即一个月中的第几天，1-31 */
  int mon;  /* 月， 从一月算起，0-11        */
  int year; /* 年， 从1900至今已经多少年    */
  int week; /* 星期                         */
};

/* 全局函数声明 */
extern void rtc_cli(int argc, char *argv[]);
extern int rtc_settime(struct TimeStructure *ptime);
extern void rtc_gettime(struct TimeStructure *ptime);
extern unsigned int rtc_getsec(void);
extern void rtc_datefromsec(struct TimeStructure *time, unsigned int sec,
                            enum en_date_type type);
extern unsigned int rtc_secfromdate(struct TimeStructure *time,
                                    enum en_date_type type);

extern void delay_us();
extern void delay_nus(uint32_t n);
#endif /* __RTC_H__ */
