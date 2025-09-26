
#ifndef __HX711_SAMPLE_H__
#define __HX711_SAMPLE_H__

#include <stdio.h>
#include <string.h>

#include "basecmd.h"         // oid_alloc
#include "board/gpio.h"      // struct gpio_in
#include "board/internal.h"  // gpio_peripheral
#include "board/irq.h"       // irq_disable
#include "board/misc.h"      // timer_from_us
#include "command.h"         // DECL_COMMAND
#include "sched.h"           // struct timer

// HX711 采样周期：us,采样滤为5ms时会存在大量 not ready, 10ms也有
#define HX711_SAMPLE_PERIOD_US \
  4700  // HX711S  最大采样率：80SPS，三个sensor分时采样
#define HX711_SAMPLE_PERIOD_TICKS \
  (CONFIG_CLOCK_FREQ / 1000000 * HX711_SAMPLE_PERIOD_US)
#define HX711_SAMPLE_REST_TICKS (HX711_SAMPLE_PERIOD_TICKS)
#define SYS_TICK_MS 1000000 / CONFIG_CLOCK_FREQ

struct Hx711Sample {
  struct timer hx711_sample_timer;
  uint32_t oid;
  uint32_t rest_ticks;
  uint8_t flags;
  uint8_t is_calibration;
  uint32_t hx711_count;
  int32_t times_read;
  struct gpio_out clks[4];
  struct gpio_in sdos[4];
  uint32_t traceflag;
  uint32_t time_stamp[4];         //采样时间戳
  int32_t init_values[4];         //静置状态的采样数据
  int32_t sample_values[4];       //采样数据
  int32_t calibration_values[4];  //标定后的采样数据
  int32_t filter_values[4];       //滤波后的数据
  int32_t fusion_filter_value;    //融合滤波后的数据
  int32_t kalman_q[4];
  int32_t kalman_r[4];
};

enum { HX711_SAMPLE_START = 1 << 0 };
extern struct Hx711Sample hx711_sample;

extern void hx711s_sample_task(void);
extern void hx711_sample_cli(int argc, char *argv[]);
extern int32_t hx711_sliding_window_avg(int index, int window, int32_t data);
#endif
