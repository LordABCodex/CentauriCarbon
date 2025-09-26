
#ifndef __HX711S_H__
#define __HX711S_H__

#include <stdio.h>
#include <string.h>

#include "basecmd.h"         // oid_alloc
#include "board/gpio.h"      // struct gpio_in
#include "board/internal.h"  // gpio_peripheral
#include "board/irq.h"       // irq_disable
#include "board/misc.h"      // timer_from_us
#include "command.h"         // DECL_COMMAND
#include "sched.h"           // struct timer
#include "stm32/hx717_exit.h"

// HX711s上报周期：us
#define HX711S_PERIOD_US 5000  // HX711S  最大采样率：80SPS
#define HX711S_PERIOD_TICKS (CONFIG_CLOCK_FREQ / 1000000 * HX711S_PERIOD_US)
#define HX711S_REST_TICKS (HX711S_PERIOD_TICKS)
#define SYS_TICK_MS 1000000 / CONFIG_CLOCK_FREQ
// PB13:29
// PC7：39
// PC6：38
// PC9：41

// PB3 ：19
#define HX717_S0 19

// PD2：50
#define HX717_S1 50

#define DEBUG_HX711 0

enum { HX711S_START = 1 << 0 };

enum SG_MODE {
  SG_MODE_HX711_FULL_BRIDGE = 0,
  SG_MODE_HX711_HALF_BRIDGE = 1,
  SG_MODE_HX717_FULL_BRIDGE = 2,
  SG_MODE_HX717_HALF_BRIDGE = 3,
};
enum PROBE_CHECK_MODE {
  PROBE_CHECK_MODE_NONE = 0,
  PROBE_CHECK_MODE_START = 1,
  PROBE_CHECK_MODE_STOP = 2,
  PROBE_CHECK_MODE_CALIBRATION = 3,
};

enum SG_STATUS {
  SG_1_OFFLINE = 1 << 0,
  SG_2_OFFLINE = 1 << 1,
  SG_3_OFFLINE = 1 << 2,
  SG_4_OFFLINE = 1 << 3,
  SG_1_REVERSE = 1 << 4,
  SG_2_REVERSE = 1 << 5,
  SG_3_REVERSE = 1 << 6,
  SG_4_REVERSE = 1 << 7,
};

struct Hx711s {
  struct timer hx711s_timer;
  uint32_t oid;
  uint32_t rest_ticks;
  uint32_t sample_period;  //采样周期
  uint32_t enable_channels;
  uint32_t enable_hpf;
  uint32_t enable_shake_filter;
 // bit7-0:       4            3           2      1             0
 //                       k补偿计算方式     回退点的方式         应用k补偿
 //             0:         0:线性拟合       00:线性             0:不应用
 //             1:         1:固定规律       01:后向阈值(从后往前)          1:应用
 //                                        10:前向阈值(从前往后)    
  uint32_t find_index_mode;     
  uint32_t heartbeat_period;
  int32_t x;
  int32_t y;
  int32_t z;
  uint8_t is_bottom_detection;
  int32_t probe_check_cmd;
  uint8_t is_running_check;
  uint8_t flags;
  uint8_t is_calibration;
  uint8_t is_trigger;
  uint8_t trigger_index;
  uint8_t now_trigger;
  uint32_t trigger_tick;
  uint32_t hx711_count;
  uint32_t sg_mode;     // 芯片方案： 0：hx711 全桥   1：hx711 半桥   2：hx717 全桥  3：hx717 半桥 
  uint8_t install_dir;  // bit: 0 1 2 3: 代表 0.1.2.3应变片的安装方向：
                        // 0负相关(下压时，值减小) 1正相关(下压时，值增大)
  uint32_t debug_data;
  int32_t times_read;
  struct gpio_out clks[4];
  struct gpio_in sdos[4];
  uint32_t traceflag;
  uint32_t time_stamp[5];         //采样时间戳: 5爲融合後的時間戳
  int32_t init_values[4];         //静置状态的采样数据
  int32_t amplitude_values[4];    //电机shake时的振幅值
  int32_t sample_values[4];       //采样数据
  int32_t max_data_num;
  int32_t calibration_values[4];  //标定后的采样数据
  int32_t filter_values[4];       //滤波后的数据
  int32_t fusion_filter_value;    //融合滤波后的数据
  int32_t kalman_q[4];
  int32_t kalman_r[4];
  int32_t max_th;
  int32_t min_th;
  int32_t th_k;
  int32_t status;  // 应变片状态字
};


extern void hx711s_cli(int argc, char *argv[]);
#endif
