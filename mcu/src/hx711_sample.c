#include "hx711_sample.h"

#include <stdio.h>

#include "local_util.h"
#include "rtc.h"
#include "tiny_ekf.h"
#include "tiny_ekf_config.h"

static struct task_wake hx711_sample_wake;
struct Hx711Sample hx711_sample;
static ekf_t hx711_sample_ekf_t;
static uint32_t hx711_sample_oid;

/**
 * @description: hx711 kalmanl滤器初始化
 * @author: your name
 * @param {ekf_t} *ekf
 * @param {Hx711sFilterParams} kalman_params
 * @return {*}
 */
static void hx711_ekf_init(ekf_t *ekf, struct Hx711Sample *h) {
  ekf->n = Nsta;
  ekf->m = Mobs;

  // Set Q
  ekf->Q[0][0] = h->kalman_q[0];

  // initial covariances of state noise, measurement noise
  ekf->P[0][0] = h->kalman_r[0];
  ekf->R[0][0] = h->kalman_r[0];

  // 初始状态变量
  ekf->x[0] = 0;

  UART1_Printf("[INFO]hx711_ekf_init m:%d n:%d x[0]:%f Q:%f P:%f R:%f\r\n",
               ekf->m, ekf->n, ekf->x[0], ekf->Q[0][0], ekf->P[0][0],
               ekf->R[0][0]);
}

/**
 * @description:hx711 模型更新
 * @author: your name
 * @param {ekf_t} *ekf
 * @param {double*} z observation vector, length <i>m</i>
 * @return {*}
 */
static void ekf_model(ekf_t *ekf) {
  // Process model is f(x) = x
  ekf->fx[0] = ekf->x[0];

  // So process model Jacobian is identity matrix
  ekf->F[0][0] = 1;

  // Measurement function simplifies the relationship between state and sensor
  // readings for convenience. A more realistic measurement function would
  // distinguish between state value and measured value; e.g.:
  //   hx[0] = pow(this->x[0], 1.03);
  //   hx[1] = 1.005 * this->x[1];
  //   hx[2] = .9987 * this->x[1] + .001;
  // Sensor acquisition value from previous state
  ekf->hx[0] = ekf->x[0];

  // Jacobian of measurement function
  ekf->H[0][0] = 1;  // Sensor acquisition value from previous state

  UART1_Printf("[DEBUG]ekf_model m:%d n:%d x[0]:%f hx[0]:%f Q:%f P:%f R:%f\r\n",
               ekf->m, ekf->n, ekf->x[0], ekf->hx[0], ekf->Q[0][0],
               ekf->P[0][0], ekf->R[0][0]);
  delay_nus(1000);
}

/**
 * @description:
 * @author: your name
 * @param {ekf_t} *ekf
 * @param {double} *z 测量值
 * @return {*}
 */
static int8_t hx711_ekf_step(ekf_t *ekf, double *z) {
  UART1_Printf("[DEBUG]hx711_ekf_step z:%f\r\n", *z);
  delay_nus(1000);
  ekf_model(ekf);
  return ekf_step(ekf, z) ? 0 : 1;
}

/**
 * @description: 滑窗均值滤波
 * @author: your name
 * @param {int} index： snesor index
 * @param {int32_t} data： snesor sample value
 * @return {int32_t} 滤波值
 */
int32_t hx711_sliding_window_avg(int index, int window, int32_t data) {
  return sliding_window_avg_filter(index, MAX_DATA_NUM, window, data, 0, 1);
}

/**
 * @description: hx711_sample 定时器事件处理
 * @author: your name
 * @param {timer} *t
 * @return {*}
 */
static uint_fast8_t hx711_sample_event(struct timer *t) {
  sched_wake_task(&hx711_sample_wake);

  struct Hx711Sample *h =
      container_of(t, struct Hx711Sample, hx711_sample_timer);

  h->hx711_sample_timer.waketime += h->rest_ticks;

  if (h->flags) {
    return SF_RESCHEDULE;
  }

  return SF_DONE;
}

/**
 * @description: hx711_sample task配置
 * @author: your name
 * @param {uint32_t} *args
 * @return {*}
 */
void command_config_hx711_sample(uint32_t *args) {
  struct Hx711Sample *h =
      oid_alloc(args[0], command_config_hx711_sample, sizeof(*h));
  h->oid = args[0];
  hx711_sample_oid = args[0];
  h->hx711_count = args[1];
  if (h->hx711_count > 4) shutdown("Max of 4 hx711");
  h->kalman_q[0] = args[2];
  h->kalman_r[0] = args[3];
  h->is_calibration = 0;
  h->init_values[0] = 0;
  h->init_values[1] = 0;
  h->init_values[2] = 0;
  h->init_values[3] = 0;

  UART1_Printf(
      "command_config_hx711_sample: oid:%d  hx711_count: %d sample period:%dus "
      "kalman_q:%d(%d) "
      "kalman_r:%d(%d)\r\n",
      h->oid, h->hx711_count, HX711_SAMPLE_PERIOD_US, h->kalman_q, args[2],
      h->kalman_r, args[3]);

  h->flags = 0;
  h->rest_ticks = HX711_SAMPLE_REST_TICKS;
  h->hx711_sample_timer.func = hx711_sample_event;
  // sendf("debug_hx711_sample oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u",
  //       (int)args[0], (int)args[0], (int)args[1], (int)args[2],
  //       (int)args[3]);

  // 本地拷贝备份
  memcpy(&hx711_sample, h, sizeof(*h));
  // 参数初始化
  hx711_ekf_init(&hx711_sample_ekf_t, h);
}
DECL_COMMAND(
    command_config_hx711_sample,
    "config_hx711_sample oid=%c hx711_count=%c kalman_q=%u kalman_r=%u");

/**
 * @description:
 * @author: your name
 * @param {Hx711Sample} *h
 * @param {uint8_t} choosed_sensor_hex: bit:
 * 7 6 5 4     3       2      1      0
 *          sensor4 sensor3 sensor2 sensor1
 * @return {*}
 */
static uint8_t hx711_sample_process(struct Hx711Sample *h,
                                    uint8_t choosed_sensor_hex) {
  uint8_t is_data_valid = 0;
  // 查询A/D转换器是否就绪: 低电平表示数据就绪
  uint8_t j = 0;
  for (j = 0; j < h->hx711_count; j++) {
    uint8_t k = j;
    if (choosed_sensor_hex & (1 << k)) {
      is_data_valid &= (~(gpio_in_read(h->sdos[j]) << k));
      h->sample_values[j] = 0;
      UART1_Printf(
          "[DEBUG]hx711s_sample hx711_count: %d choosed_sensor_hex: %d "
          "h->sdos[j]:%d j:%d  "
          "is_data_valid:%d "
          "\r\n",
          h->hx711_count, choosed_sensor_hex, h->sdos[j], j, is_data_valid);
    } else {
      UART1_Printf("not choosed j: %d choosed_sensor_hex:%d\r\n", j,
                   choosed_sensor_hex);
    }
  }

  if (is_data_valid == 0xFF) {
    // 置低SCK
    for (j = 0; j < h->hx711_count; j++) {
      uint8_t k = j;
      if (choosed_sensor_hex & (1 << k)) {
        gpio_out_write(h->clks[j], 0);
      }
    }
    uint8_t i = 0;
    for (i = 0; i < 24; i++) {
      // SCK上升沿
      for (j = 0; j < h->hx711_count; j++) {
        uint8_t k = j;
        if (choosed_sensor_hex & (1 << k)) {
          gpio_out_write(h->clks[j], 1);
        }
      }
      // 数据位初始化为1
      for (j = 0; j < h->hx711_count; j++) {
        uint8_t k = j;
        if (choosed_sensor_hex & (1 << k)) {
          h->sample_values[j] = h->sample_values[j] << 1;
        }
      }
      // 置低SCK
      for (j = 0; j < h->hx711_count; j++) {
        uint8_t k = j;
        if (choosed_sensor_hex & (1 << k)) {
          gpio_out_write(h->clks[j], 0);
        }
      }
      // 下降沿读取数据位
      for (j = 0; j < h->hx711_count; j++) {
        uint8_t k = j;
        if (choosed_sensor_hex & (1 << k)) {
          h->sample_values[j] += (gpio_in_read(h->sdos[j]) > 0 ? 1 : 0);
        }
      }
    }
    // 默认开启下一次转换：若需要修改通道或者增益，需要再发2-4个数据
    for (j = 0; j < h->hx711_count; j++) {
      uint8_t k = j;
      if (choosed_sensor_hex & (1 << k)) {
        gpio_out_write(h->clks[j], 1);
      }
    }
    // 符号位转换
    for (j = 0; j < h->hx711_count; j++) {
      uint8_t k = j;
      if (choosed_sensor_hex & (1 << k)) {
        h->sample_values[j] |=
            ((h->sample_values[j] & 0x00800000) != 0 ? 0xFF000000 : 0);
      }
    }
    for (j = 0; j < h->hx711_count; j++) {
      uint8_t k = j;
      if (choosed_sensor_hex & (1 << k)) {
        gpio_out_write(h->clks[j], 0);
      }
    }
    if (h->traceflag & 0x02) {
      UART1_Printf("[SENSOR]%d: %d %d %d %d\r\n", choosed_sensor_hex,
                   h->sample_values[0], h->sample_values[1],
                   h->sample_values[2], h->sample_values[3]);
    }
    return 1;
  } else {
    UART1_Printf("[WARNING]%d not ready.\r\n", choosed_sensor_hex);
    return 0;
  }
  return 1;
}

/**
 * @description: 采集单通道
 * @author: your name
 * @param {uint8_t} choosed_sensor_hex: bit:
 * 7 6 5 4     3       2      1      0
 *          sensor4 sensor3 sensor2 sensor1
 * @return {*}
 */
static uint8_t hx711_sample_one(struct Hx711Sample *h, uint8_t choosed_sensor) {
  // irq_disable();
  static int32_t loop = 0;
  loop++;
  uint8_t is_data_valid = 1;
  // 查询A/D转换器是否就绪: 低电平表示数据就绪
  int wait_cycle = HX711_SAMPLE_PERIOD_US / 1000 - 2;
  if (wait_cycle < 1) {
    wait_cycle = 1;
  }
  // for (int i = 0; i < wait_cycle; i++) {
  // for (int i = 0; i < 20; i++) {
  is_data_valid = ((gpio_in_read(h->sdos[choosed_sensor]) > 0) ? 1 : 0);
  //   if (is_data_valid) {
  //     break;
  //   } else {
  //     delay_nus(HX711_SAMPLE_PERIOD_US / 10);
  //   }
  // }
  if (is_data_valid == 0) {
    h->sample_values[choosed_sensor] = 0;
    // 置低SCK
    // delay_nus(1);
    gpio_out_write(h->clks[choosed_sensor], 0);
    // delay_nus(1);
    uint8_t i = 0;
    for (i = 0; i < 24; i++) {
      // SCK上升沿
      gpio_out_write(h->clks[choosed_sensor], 1);
      // delay_nus(1);
      // 数据位初始化为1
      h->sample_values[choosed_sensor] =
          (h->sample_values[choosed_sensor] << 1);
      // 置低SCK
      gpio_out_write(h->clks[choosed_sensor], 0);
      // delay_nus(1);
      // 下降沿读取数据位
      uint8_t tmp = (gpio_in_read(h->sdos[choosed_sensor]) > 0 ? 1 : 0);
      h->sample_values[choosed_sensor] += tmp;
    }
    // delay_nus(1);
    // 默认开启下一次转换：若需要修改通道或者增益，需要再发2-4个数据
    gpio_out_write(h->clks[choosed_sensor], 1);
    // delay_nus(1);
    // 符号位转换
    h->sample_values[choosed_sensor] |=
        ((h->sample_values[choosed_sensor] & 0x00800000) != 0 ? 0xFF000000 : 0);
    // 置0
    gpio_out_write(h->clks[choosed_sensor], 0);
    // if (h->traceflag & 0x02) {
    //   UART1_Printf("[SENSOR]%d %d\r\n", choosed_sensor,
    //                h->sample_values[choosed_sensor]);
    // }
    // irq_enable();
    return 1;
  } else {
    // irq_enable();
    if (h->traceflag & 0x80) {
      uint32_t rtc_tick = rtc_getsec();
      UART1_Printf("[WARNING]%d %d not ready. %d\r\n", loop, choosed_sensor,
                   rtc_tick);
    }
    return 0;
  }
  // irq_enable();
  return 1;
}

/**
 * @description:
 * @author: your name
 * @param {uint32_t} *args
 * @return {*}
 */
void command_add_hx711_sample(uint32_t *args) {
  uint8_t oid = args[0];
  uint8_t index = args[1];
  struct Hx711Sample *h = oid_lookup(oid, command_config_hx711_sample);
  if (index >= h->hx711_count) shutdown("Set hx711 past maximum count");

  h->clks[index] = gpio_out_setup(args[2], 0);
  h->sdos[index] = gpio_in_setup(args[3], 0);

  // sendf("debug_hx711_sample oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u",
  //       (int)args[0], (int)args[0], (int)args[1], (int)args[2],
  //       (int)args[3]);
  memcpy(&hx711_sample, h, sizeof(*h));
  UART1_Printf(
      "[INFO]command_add_hx711_sample oid=%d arg[0]=%u arg[1]=%u arg[2]=%u "
      "arg[3]=%u\r\n",
      (int)args[0], (int)args[0], (int)args[1], (int)args[2], (int)args[3]);
}
DECL_COMMAND(command_add_hx711_sample,
             "add_hx711_sample oid=%c index=%c clk_pin=%u sdo_pin=%u");

/**
 * @description: 初始上电，标定应变片初始采集值
 * @author: your name
 * @param {Hx711Sample} *h
 * @return {*}
 */
static bool calibration_strain_gauge_init_value(struct Hx711Sample *h) {
  int32_t sum[4] = {0};
  int32_t sample_num[4] = {0};
  for (int i = 0; i < h->times_read; i++) {
    for (int j = 0; j < h->hx711_count; j++) {
      if (!hx711_sample_one(h, j)) {
        UART1_Printf("[WARNING]hx711s_sample_task hx711s_sample %d failed\r\n",
                     j);
      } else {
        sum[j] += h->sample_values[j];
        sample_num[j]++;
        UART1_Printf("[INFO] hx711: %d value: %d  sum: %d  sum_num: %d\r\n", j,
                     h->sample_values[j], sum[j], sample_num[j]);
      }
      delay_nus(5000);
    }
  }
  h->is_calibration = 1;

  for (int j = 0; j < h->hx711_count; j++) {
    if (sample_num[j] > 0) {
      h->init_values[j] = sum[j] / sample_num[j];
    } else {
      UART1_Printf("[ERROR]  %d calibration failed\r\n", j);
      h->is_calibration = 0;
      return FALSE;
    }
  }
  UART1_Printf(
      "[INFO] calibration times_read: %d sensor init_value: %d %d %d %d\r\n",
      h->times_read, h->init_values[0], h->init_values[1], h->init_values[2],
      h->init_values[3]);
  return TRUE;
}

/**
 * @description: 初始上电，标定应变片初始采集值
 * @author: your name
 * @param {Hx711Sample} *h
 * @return {*}
 */
static bool calibration_strain_gauge_init_value_new(struct Hx711Sample *h,
                                                    uint8_t choosed_sensor) {
  static int32_t sum[4] = {0};
  static int32_t sample_num[4] = {0};
  static int32_t sample_values[4][200] = {0};

  if ((choosed_sensor + 1) == h->hx711_count) {
    h->times_read--;
  }

  if (h->times_read == 0) {
    h->is_calibration = 1;
    for (int j = 0; j < h->hx711_count; j++) {
      if (sample_num[j] > 0) {
        h->init_values[j] = sum[j] / sample_num[j];  //均值滤波
        h->init_values[j] =
            median_filter(sample_values[j], sample_num[j]);  //中值滤波
      } else {
        UART1_Printf("[ERROR]  %d calibration failed\r\n", j);
        h->is_calibration = 0;
        sample_num[j] = 0;
        sum[j] = 0;
        return FALSE;
      }
      sample_num[j] = 0;
      sum[j] = 0;
    }
    UART1_Printf(
        "[INFO] calibration times_read: %d sensor init_value: %d %d %d %d\r\n",
        h->times_read, h->init_values[0], h->init_values[1], h->init_values[2],
        h->init_values[3]);
    return TRUE;
  }

  if (!hx711_sample_one(h, choosed_sensor)) {
    UART1_Printf("[WARNING]hx711s_sample_task hx711s_sample %d failed\r\n",
                 choosed_sensor);
  } else {
    sum[choosed_sensor] += h->sample_values[choosed_sensor];
    sample_values[choosed_sensor][sample_num[choosed_sensor]] =
        h->sample_values[choosed_sensor];
    sample_num[choosed_sensor]++;
    UART1_Printf("[INFO] hx711: %d value: %d  sum: %d  sum_num: %d\r\n",
                 choosed_sensor, h->sample_values[choosed_sensor],
                 sum[choosed_sensor], sample_num[choosed_sensor]);
  }
  return TRUE;
}

/**
 * @description: 标定应变片，静置状态下采集应变片的初始值
 * @author: your name
 * @param {uint32_t} *args
 * @return {*}
 */
void command_calibration_hx711_sample(uint32_t *args) {
  uint8_t oid = args[0];
  struct Hx711Sample *h = oid_lookup(oid, command_config_hx711_sample);
  h->times_read = args[1];

  if (h->times_read < 1) {
    h->times_read = 50;
  } else if (h->times_read > 200) {
    h->times_read = 200;
  }
  UART1_Printf("command_calibration_hx711_sample, times_read:%d\r\n",
               h->times_read);
  h->is_calibration = 0;
  h->init_values[0] = 0;
  h->init_values[1] = 0;
  h->init_values[2] = 0;
  h->init_values[3] = 0;

  // 开启任务
  h->flags |= HX711_SAMPLE_START;
  sched_del_timer(&h->hx711_sample_timer);
  irq_disable();
  h->hx711_sample_timer.waketime = timer_read_time() + h->rest_ticks;
  sched_add_timer(&h->hx711_sample_timer);
  // h->traceflag |= 0x06;
  irq_enable();
  memcpy(&hx711_sample, h, sizeof(*h));
  UART1_Printf("[INFO]start hx711 sample task...\r\n");

  // if (calibration_strain_gauge_init_value(h)) {
  //   // 开启任务
  //   h->flags |= HX711_SAMPLE_START;
  //   sched_del_timer(&h->hx711_sample_timer);
  //   irq_disable();
  //   h->hx711_sample_timer.waketime = timer_read_time() + h->rest_ticks;
  //   sched_add_timer(&h->hx711_sample_timer);
  //   // h->traceflag |= 0x06;
  h->traceflag |= 0x80;
  //   irq_enable();
  //   memcpy(&hx711_sample, h, sizeof(*h));
  //   UART1_Printf("[INFO]start hx711 sample task...\r\n");
  // } else {
  //   UART1_Printf("[ERROR]calibration failed\r\n");
  // }
}
DECL_COMMAND(command_calibration_hx711_sample,
             "calibration_sample oid=%c times_read=%hu");

/**
 * @description: hx711采样滤波任务：伴随 sched
 * 调度任务时间片周期执行,分时采样每个应变传感器，同时进行滤波融合处理
 * @author: your name
 * @return {*}
 */
void hx711_sample_task(void) {
  static uint8_t last_choosed_sensor = 0;
  static uint32_t last_rtc_tick = 0;
  static uint32_t loop = 0;
  static int32_t last_calibration_values[4] = {0};

  loop++;
  if (!sched_check_wake(&hx711_sample_wake)) {
    return;
  }

  struct Hx711Sample *h =
      oid_lookup(hx711_sample_oid, command_config_hx711_sample);

  if (!(h->flags & HX711_SAMPLE_START)) {
    UART1_Printf("[WARNING]hx711_sample_task not start\r\n");
    return;
  }

  // 1.选择sensor
  uint8_t chuoosed_sensor = last_choosed_sensor + 1;
  if (chuoosed_sensor > 2) {
    chuoosed_sensor = 0;
  }
  last_choosed_sensor = chuoosed_sensor;

  // 2.采样
  uint32_t now_tick = rtc_getsec();
  last_rtc_tick = now_tick;
  uint8_t chuoosed_sensor_hex = (0x01 << (chuoosed_sensor));

  // 未标定则先标定
  if (!hx711_sample.is_calibration) {
    calibration_strain_gauge_init_value_new(&hx711_sample, chuoosed_sensor);
  } else {
    if (!hx711_sample_one(&hx711_sample, chuoosed_sensor)) {
      // UART1_Printf("[WARNING]hx711s_sample_task hx711s_sample %d failed\r\n",
      //              chuoosed_sensor);
    }
    // 3.kalman滤波
    else {
      hx711_sample.time_stamp[chuoosed_sensor] = now_tick;
      int32_t fliter_value = 0;

      hx711_sample.calibration_values[chuoosed_sensor] =
          hx711_sample.sample_values[chuoosed_sensor] -
          hx711_sample.init_values[chuoosed_sensor];

      // 滑动窗口滤波
      int32_t sliding_filter_value = hx711_sliding_window_avg(
          chuoosed_sensor, 5, hx711_sample.calibration_values[chuoosed_sensor]);

      int32_t tmp = hx711_sample.calibration_values[chuoosed_sensor] -
                    sliding_filter_value;

      if (tmp > 100000 || tmp < -100000) {
        UART1_Printf(
            "[WARNING]%d:exception-value:%d sliding_filter_value:%d at:%d\r\n",
            chuoosed_sensor, tmp, sliding_filter_value, now_tick);
        hx711_sample.calibration_values[chuoosed_sensor] =
            last_calibration_values[chuoosed_sensor];
      } else {
        double z = hx711_sample.calibration_values[chuoosed_sensor];
        // 滤波函数
        // if (hx711_ekf_step(&hx711_sample_ekf_t, &z)) {
        //   fliter_value = hx711_sample_ekf_t.x[0];
        // } else {
        //   UART1_Printf("[ERROR]hx711s_sample_task hx711_ekf_step %d
        //   failed\r\n",
        //                chuoosed_sensor);
        // }
        hx711_sample.filter_values[chuoosed_sensor] = fliter_value;
        hx711_sample.fusion_filter_value =
            hx711_sample.calibration_values[chuoosed_sensor];
        //选中的sensor 采样值 融合滤波置 tick
        if (hx711_sample.traceflag & 0x04) {
          UART1_Printf(
              "%d %d %d %d %d %d %d %d %d %d %d %d %d\r\n", loop,
              chuoosed_sensor, hx711_sample.calibration_values[chuoosed_sensor],
              fliter_value, hx711_sample.calibration_values[0],
              hx711_sample.calibration_values[1],
              hx711_sample.calibration_values[2],
              hx711_sample.calibration_values[3], hx711_sample.sample_values[0],
              hx711_sample.sample_values[1], hx711_sample.sample_values[2],
              hx711_sample.sample_values[3], now_tick);
        }
      }
      last_calibration_values[chuoosed_sensor] = sliding_filter_value;
    }
  }
}
DECL_TASK(hx711s_sample_task);

/**
 * @description:
 * @author: your name
 * @return {*}
 */
void hx711_sample_shutdown(void) {
  uint8_t oid;
  struct Hx711Sample *h;
  foreach_oid(oid, h, command_config_hx711_sample) {
    h->times_read = 0;
    h->flags &= ~HX711_SAMPLE_START;
  }
  return;
}
DECL_SHUTDOWN(hx711_sample_shutdown);

/**
 * @description: Hx711 sample命令行调试接口
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void hx711_sample_cli(int argc, char *argv[]) {
  //参数配置

  if (0 == strcmp("-config", argv[0])) {
    if (0 == strcmp("-show", argv[1])) {
      UART1_Printf("\r\n");
      UART1_Printf("\r\n HX711S Config      :");
      UART1_Printf("\r\n oid                : %d", hx711_sample.oid);
      UART1_Printf("\r\n rest_ticks         : %d", hx711_sample.rest_ticks);
      UART1_Printf("\r\n flags              : %d", hx711_sample.flags);
      UART1_Printf("\r\n hx711_count        : %d", hx711_sample.hx711_count);
      UART1_Printf("\r\n times_read         : %d", hx711_sample.times_read);
      UART1_Printf("\r\n waketime           : %d",
                   hx711_sample.hx711_sample_timer.waketime);
      UART1_Printf("\r\n kalman_q           : %d %d %d %d",
                   hx711_sample.kalman_q[0], hx711_sample.kalman_q[1],
                   hx711_sample.kalman_q[2], hx711_sample.kalman_q[3]);
      UART1_Printf("\r\n kalman_r           : %d %d %d %d",
                   hx711_sample.kalman_r[0], hx711_sample.kalman_r[1],
                   hx711_sample.kalman_r[2], hx711_sample.kalman_r[3]);
      UART1_Printf("\r\n is_calibration     : %d", hx711_sample.is_calibration);
      UART1_Printf("\r\n sensor init value  : %d %d %d %d",
                   hx711_sample.init_values[0], hx711_sample.init_values[1],
                   hx711_sample.init_values[2], hx711_sample.init_values[3]);
      UART1_Printf("\r\n hx711_sample_ekf_t : %d %d %f %f %f",
                   hx711_sample_ekf_t.m, hx711_sample_ekf_t.n,
                   hx711_sample_ekf_t.P[0][0], hx711_sample_ekf_t.Q[0][0],
                   hx711_sample_ekf_t.R[0][0]);

    } else if (0 == strcmp("-set", argv[1])) {
      if (0 == strcmp("-rest_ticks", argv[2])) {
        hx711_sample.rest_ticks = atoi(argv[3]);
        UART1_Printf("\r\n rest_ticks     : %d", hx711_sample.rest_ticks);
      } else if (0 == strcmp("-hx711_count", argv[2])) {
        hx711_sample.hx711_count = atoi(argv[3]);
        UART1_Printf("\r\n hx711_count     : %d", hx711_sample.hx711_count);
      } else if (0 == strcmp("-times_read", argv[2])) {
        hx711_sample.times_read = atoi(argv[3]);
        UART1_Printf("\r\n times_read     : %d", hx711_sample.times_read);
      } else if (0 == strcmp("-waketime", argv[2])) {
        hx711_sample.hx711_sample_timer.waketime = atoi(argv[3]);
        UART1_Printf("\r\n waketime     : %d",
                     hx711_sample.hx711_sample_timer.waketime);
      } else if (0 == strcmp("-kalman_q", argv[2])) {
        hx711_sample.kalman_q[0] = atoi(argv[3]);
        hx711_sample.kalman_q[1] = atoi(argv[4]);
        hx711_sample.kalman_q[2] = atoi(argv[5]);
        hx711_sample.kalman_q[3] = atoi(argv[6]);
        UART1_Printf("\r\n kalman_q:  %d %d %d %d ", hx711_sample.kalman_q[0],
                     hx711_sample.kalman_q[1], hx711_sample.kalman_q[2],
                     hx711_sample.kalman_q[3]);

        // 参数重置
        hx711_ekf_init(&hx711_sample_ekf_t, &hx711_sample);
      } else if (0 == strcmp("-kalman_r", argv[2])) {
        hx711_sample.kalman_r[0] = atoi(argv[3]);
        hx711_sample.kalman_r[1] = atoi(argv[4]);
        hx711_sample.kalman_r[2] = atoi(argv[5]);
        hx711_sample.kalman_r[3] = atoi(argv[6]);

        // 参数重置
        hx711_ekf_init(&hx711_sample_ekf_t, &hx711_sample);
        UART1_Printf("\r\n kalman_r:  %d %d %d %d ", hx711_sample.kalman_r[0],
                     hx711_sample.kalman_r[1], hx711_sample.kalman_r[2],
                     hx711_sample.kalman_r[3]);
      } else {
        UART1_Printf("\r\nNOTICE: hx711_cli para:%s err.", argv[2]);
      }
    } else {
      UART1_Printf("\r\nNOTICE: hx711_cli -config:%s err.", argv[1]);
    }
  }
  // 数据
  else if (0 == strcmp("-data", argv[0])) {
    UART1_Printf("\r\nsensor1 value       : %d", hx711_sample.sample_values[0]);
    UART1_Printf("\r\nsensor2 value       : %d", hx711_sample.sample_values[1]);
    UART1_Printf("\r\nsensor3 value       : %d", hx711_sample.sample_values[2]);
    UART1_Printf("\r\nsensor4 value       : %d", hx711_sample.sample_values[3]);
    UART1_Printf("\r\nsensor1 filter value: %d", hx711_sample.filter_values[0]);
    UART1_Printf("\r\nsensor2 filter value: %d", hx711_sample.filter_values[1]);
    UART1_Printf("\r\nsensor3 filter value: %d", hx711_sample.filter_values[2]);
    UART1_Printf("\r\nsensor4 filter value: %d", hx711_sample.filter_values[3]);
    UART1_Printf("\r\nfusion filter value : %d",
                 hx711_sample.fusion_filter_value);
  }
  // 开启采集
  else if (0 == strcmp("-start", argv[0])) {
    if (0 == strcmp("-once", argv[1])) {
      UART1_Printf("\r\nsensor1 value: %d", hx711_sample.sample_values[0]);
      UART1_Printf("\r\nsensor2 value: %d", hx711_sample.sample_values[1]);
      UART1_Printf("\r\nsensor3 value: %d", hx711_sample.sample_values[2]);
      UART1_Printf("\r\nsensor4 value: %d", hx711_sample.sample_values[3]);
      UART1_Printf("\r\nfilter  value: %d", hx711_sample.sample_values[3]);

    } else if (0 == strcmp("-number", argv[1])) {
      int num = atoi(argv[4]);
      hx711_sample.times_read = num;
      hx711_sample.flags |= HX711_SAMPLE_START;
      sched_del_timer(&hx711_sample.hx711_sample_timer);
      irq_disable();
      hx711_sample.hx711_sample_timer.waketime =
          timer_read_time() + hx711_sample.rest_ticks;
      sched_add_timer(&hx711_sample.hx711_sample_timer);
    } else {
      UART1_Printf("\r\nNOTICE: hx711_sample_cli para:%s err.", argv[1]);
    }

  }
  // 停止采集
  else if (0 == strcmp("-stop", argv[0])) {
  }
  // 指针地址
  else if (0 == strcmp("-address", argv[0])) {
    UART1_Printf("\r\np_hx711s address: 0x%x", hx711_sample);
  }
  // 调试日志
  else if (0 == strcmp("-trace", argv[0])) {
    if (0 == strcmp("-sample_task", argv[1])) {
      hx711_sample.traceflag = atoi(argv[2]);
      UART1_Printf("\r\nhx711s_sample_data.traceflag:%d",
                   hx711_sample.traceflag);
    } else {
      hx711_sample.traceflag = !hx711_sample.traceflag;
      UART1_Printf("\r\nhx711_sample_cli.traceflag:%d", hx711_sample.traceflag);
    }
  } else {
    UART1_Printf("\r\nNOTICE: hx711_sample_cli para:%s err.", argv[0]);
  }

  // UART1_Printf("\r\n");
  // struct Hx711Sample *h =
  //     oid_lookup(hx711_sample_oid, command_config_hx711_sample);
  // UART1_Printf("\r\n");
}
