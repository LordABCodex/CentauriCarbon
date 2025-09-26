#include "rtc.h"

//月修正数据表
const char table_week[12] = {0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 5};

//平年的月份日期表
const unsigned int mon_table[12] = {31, 28, 31, 30, 31, 30,
                                    31, 31, 30, 31, 30, 31};

void delay(unsigned int delay) {
  unsigned int i, j;

  for (i = 0; i < delay; i++) {
    for (j = 0xFF; j != 0; j--) {
      ;
    }
    // IWDG_Feed();
    ;
  }
}

/**
 * @description: 判断是否是闰年
          四年一闰，百年不闰，四百年再闰
 * @author: your name
 * @param {int} year
 * @return {*}
 */
int rtc_isleap(int year) {
  if (0 == (year % 4)) {
    if (0 == (year % 100)) {
      if (0 == (year % 400)) {
        return 1;
      } else {
        return 0;
      }
    } else {
      return 1;
    }
  } else {
    return 0;
  }
}

/**
 * @description: sec自1970年1月1日00:00:00的秒数, 转换日期的格式要给定
 * @author: your name
 * @return {*}
 */
void rtc_datefromsec(struct TimeStructure *time, unsigned int sec,
                     enum en_date_type type) {
  unsigned int temp;

  /* 北京时间 = UTC时间 + 8小时 */
  if (DATE_BEIJING == type) {
    sec += 3600 * 8;
  }

  temp = sec / 86400;  //得到天数(秒钟数对应的)

  time->year = 1970;  //从1970年开始
  while (365 <= temp) {
    if (1 == rtc_isleap(time->year))  //是闰年
    {
      if (366 <= temp) {
        temp -= 366;  //闰年的秒钟数
      } else {
        break;
      }
    } else {
      temp -= 365;  //平年
    }
    time->year++;
  }

  time->mon = 0;
  while (28 <= temp)  //超过了一个月
  {
    //当年是不是闰年/2月份
    if ((1 == rtc_isleap(time->year)) && (1 == time->mon)) {
      if (29 <= temp) {
        temp -= 29;  //闰年的秒钟数
      } else {
        break;
      }
    } else {
      if (temp >= mon_table[time->mon]) {
        temp -= mon_table[time->mon];  //平年
      } else {
        break;
      }
    }
    time->mon++;
  }
  time->mon += 1;         //得到月份
  time->date = temp + 1;  //得到日期

  temp = sec % 86400;              //得到秒钟数
  time->hour = temp / 3600;        //小时
  time->min = (temp % 3600) / 60;  //分钟
  time->sec = (temp % 3600) % 60;  //秒钟
}

/**
 * @description: UNIX时间戳
          即从1970年1月1日00:00:00到现在的秒数,
          注意这个是UTC时间, 不是北京时间;
          再加8个小时即是北京时间;
 * @author: your name
 * @param {TimeStructure} *time
 * @param {enum en_date_type} type
 * @return {*}
 */
unsigned int rtc_secfromdate(struct TimeStructure *time,
                             enum en_date_type type) {
  int i;
  unsigned int seccount = 0;

  for (i = 1970; i < time->year; i++)  //把所有年份的秒钟相加
  {
    if (1 == rtc_isleap(i)) {
      seccount += 31622400;  //闰年的秒钟数
    } else {
      seccount += 31536000;  //平年的秒钟数
    }
  }

  /* 把前面月份的秒钟数相加 */
  time->mon -= 1;
  for (i = 0; i < time->mon; i++) {
    /* 月份秒钟数相加 */
    seccount += mon_table[i] * 86400;

    /* 闰年2月份增加一天的秒钟数 */
    if ((1 == rtc_isleap(time->year)) && (1 == i)) {
      seccount += 86400;
    }
  }

  seccount +=
      ((unsigned int)(time->date - 1)) * 86400;  //把前面日期的秒钟数相加
  seccount += ((unsigned int)time->hour) * 3600;  //小时秒钟数
  seccount += ((unsigned int)time->min) * 60;     //分钟秒钟数
  seccount += (unsigned int)time->sec;            //最后的秒钟加上去

  /* 北京时间 = UTC时间 + 8小时 */
  if (DATE_BEIJING == type) {
    seccount -= 3600 * 8;
  }

  return seccount;
}

/**
 * @description:
 * @author: your name
 * @param {TimeStructure} *time
 * @return {*}
 */
int rtc_dateisok(struct TimeStructure *time) {
  if (!time) {
    return 0;
  }

  /* 年1970-2038 */
  if ((YEAR_ALIGN > time->year) || (YEAR_MAX < time->year)) {
    UART1_Printf("\r\nERROR: year[%d] is err", time->year);
    return 0;
  }

  /* 月1-12 */
  if ((1 > time->mon) || (12 < time->mon)) {
    UART1_Printf("\r\nERROR: mon[%d] is err", time->mon);
    return 0;
  }

  /* 日1-31 */
  if ((1 > time->date) || (31 < time->date)) {
    UART1_Printf("\r\nERROR: date[%d] is err", time->date);
    return 0;
  }

  /* 时0-23 */
  if ((0 > time->hour) || (23 < time->hour)) {
    UART1_Printf("\r\nERROR: hour[%d] is err", time->hour);
    return 0;
  }

  /* 分0-59 */
  if ((0 > time->min) || (59 < time->min)) {
    UART1_Printf("\r\nERROR: min[%d] is err", time->min);
    return 0;
  }

  /* 秒0-59 */
  if ((0 > time->sec) || (59 < time->sec)) {
    UART1_Printf("\r\nERROR: sec[%d] is err", time->sec);
    return 0;
  }

  return 1;
}

/**
 * @description: 获取时间
 * @author: your name
 * @param {TimeStructure} *ptime
 * @return {*}
 */
void rtc_gettime(struct TimeStructure *ptime) {
  RTC_TimeTypeDef RTC_TimeTypeInitStructure;
  RTC_DateTypeDef RTC_DateTypeInitStructure;

  RTC_GetTime(RTC_Format_BIN, &RTC_TimeTypeInitStructure);
  RTC_GetDate(RTC_Format_BIN, &RTC_DateTypeInitStructure);

  ptime->year = RTC_DateTypeInitStructure.RTC_Year + YEAR_ALIGN;
  ptime->mon = RTC_DateTypeInitStructure.RTC_Month;
  ptime->date = RTC_DateTypeInitStructure.RTC_Date;
  ptime->week = RTC_DateTypeInitStructure.RTC_WeekDay;

  ptime->hour = RTC_TimeTypeInitStructure.RTC_Hours;
  ptime->min = RTC_TimeTypeInitStructure.RTC_Minutes;
  ptime->sec = RTC_TimeTypeInitStructure.RTC_Seconds;
}

/**
 * @description: rtc模块初始化:
          在出厂、调试、电池更换等场合才配置;
          有外部始终，则主要是配置外部32.768K晶振时钟作为rtc工作的时钟;
          否则使用外部高速时钟
 * @author: your name
 * @param {TimeStructure} *ptime
 * @return {*}
 */
int rtc_settime(struct TimeStructure *ptime) {
  int ret;
  int timeout = 0;
  RTC_InitTypeDef RTC_InitStructure;
  RTC_TimeTypeDef RTC_TimeTypeInitStructure;
  RTC_DateTypeDef RTC_DateTypeInitStructure;

  /* 判断日期是否正确 */
  ret = rtc_dateisok(ptime);
  if (1 != ret) {
    UART1_Printf("ERROR: rtc_settime time err.\r\n");
    return RET_FAIL;
  }

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_BackupAccessCmd(ENABLE); /*使能后备寄存器访问*/

  // RCC_LSEConfig(RCC_LSE_ON);  // 若有32.768k 外部时钟 则 LSE 开启

  //检查指定的RCC标志位设置与否,等待低速晶振就绪
  while (RESET == RCC_GetFlagStatus(RCC_FLAG_HSERDY)) {
    delay(1000);
    timeout++;
    if (1000 <= timeout) {
      UART1_Printf("ERROR: rtc_settime RCC_GetFlagStatus err.\r\n");
      return RET_FAIL;
    }
  }

  RCC_RTCCLKConfig(
      RCC_RTCCLKSource_HSE_Div2);  // 设置RTC时钟(RTCCLK),选择LSE作为RTC时钟
                                   // -- 42000000 = 42 * 1000  --> 84 *1000  -->
                                   // 21*1000
  RCC_RTCCLKCmd(ENABLE);  // 使能RTC时钟
  RTC_WaitForSynchro();   // 等待RTC APB寄存器同步

  RTC_InitStructure.RTC_AsynchPrediv =
      0x14;  // (十进制41); 0x7F(十进制127)  // RTC异步分频系数(1~0X7F)
  RTC_InitStructure.RTC_SynchPrediv =
      0x3E7;  // (十进制999);  0xFF(十进制255); ;   // RTC同步分频系数(0~7FFF)
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;  // RTC设置为,24小时格式
  ret = RTC_Init(&RTC_InitStructure);

  /* 设置日期 */
  RTC_DateTypeInitStructure.RTC_Date = ptime->date;
  RTC_DateTypeInitStructure.RTC_Month = ptime->mon;
  RTC_DateTypeInitStructure.RTC_WeekDay = ptime->week;
  if (YEAR_ALIGN < ptime->year) {
    RTC_DateTypeInitStructure.RTC_Year = ptime->year - YEAR_ALIGN;
  }
  ret = RTC_SetDate(RTC_Format_BIN, &RTC_DateTypeInitStructure);
  ASSERT(SUCCESS != ret, ERROR);

  /* 设置时间 */
  RTC_TimeTypeInitStructure.RTC_Hours = ptime->hour;
  RTC_TimeTypeInitStructure.RTC_Minutes = ptime->min;
  RTC_TimeTypeInitStructure.RTC_Seconds = ptime->sec;
  if (12 > ptime->hour) {
    RTC_TimeTypeInitStructure.RTC_H12 = RTC_H12_AM;
  } else {
    RTC_TimeTypeInitStructure.RTC_H12 = RTC_H12_PM;
  }

  ret = RTC_SetTime(RTC_Format_BIN, &RTC_TimeTypeInitStructure);
  ASSERT(SUCCESS != ret, ERROR);

  UART1_Printf("[RTC_SetTime]%d/%d/%d %d:%d:%d\r\n", ptime->year, ptime->mon,
               ptime->date, ptime->hour, ptime->min, ptime->sec);

  return RET_SUCCESS;
}

/**
 * @description: 获取系统时间 秒数 ms
 * @author: your name
 * @return {*}
 */
unsigned int rtc_getsec(void) {
  struct TimeStructure time;

  rtc_gettime(&time);
  return rtc_secfromdate(&time, DATE_BEIJING);
}

/**
 * @description: rtc模块调试接口
 * @author: your name
 * @param {int} argc
 * @param {char} *argv
 * @return {*}
 */
void rtc_cli(int argc, char *argv[]) {
  struct TimeStructure time;

  if (0 == strcmp("-s", argv[0])) {
  } else if (0 == strcmp("-gettime", argv[0])) {
    rtc_gettime(&time);
    UART1_Printf("\r\n %d/%d/%d %d:%d:%d", time.year, time.mon, time.date,
                 time.hour, time.min, time.sec);
  } else if (0 == strcmp("-settime", argv[0])) {
    time.year = strtoul(argv[1], NULL, 10);
    time.mon = strtoul(argv[2], NULL, 10);
    time.date = strtoul(argv[3], NULL, 10);
    time.hour = strtoul(argv[4], NULL, 10);
    time.min = strtoul(argv[5], NULL, 10);
    time.sec = strtoul(argv[6], NULL, 10);
    time.week = strtoul(argv[7], NULL, 10);
    rtc_settime(&time);
  } else if (0 == strcmp("-sec", argv[0])) {
    UART1_Printf("\r\nSEC: %ds", rtc_getsec());
  } else if (0 == strcmp("-sectodate", argv[0])) {
    rtc_datefromsec(&time, strtoul(argv[1], NULL, 10), DATE_BEIJING);
    UART1_Printf("\r\n %d/%d/%d %d:%d:%d", time.year, time.mon, time.date,
                 time.hour, time.min, time.sec);
  }
}

/**
 * @description: 84个__NOP();延时1us
 * @author: your name
 * @return {*}
 */
void delay_us() {
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}
/**
 * @description: 延时 n us
 * @author: your name
 * @param {uint32_t} n
 * @return {*}
 */
void delay_nus(uint32_t n) {
  while (n--) {
    delay_us();
  }
}