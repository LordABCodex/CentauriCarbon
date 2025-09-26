#include "hx711s.h"

#include <stdio.h>

#include "filter.h"
#include "hx711s.h"
#include "rtc.h"

static struct task_wake hx711s_wake;
static struct Hx711s hx711s;
static uint32_t ori_rest_ticks;
// 斜率存储点
static int32_t all_point_k[37][4];
static int32_t all_point_k_fix_index[37][4];
static int32_t current_point_index;
static int32_t current_point_index_count=0;
static int32_t fix_out_index = 0;
static int32_t out_index = 0;
static int32_t kk = 0;

static int32_t k_slope;       //中位数斜率
static int32_t bias_slope;   //斜率与回退点的倍率

#define DEBUG_DATA_NUM 3000

static int32_t sample_x;
static int32_t sample_y;

int32_t debug_fusion_data[DEBUG_DATA_NUM];
int32_t debug_data_count;
int32_t debug_trigger_index;
int32_t debug_trigger_data_value;
int32_t debug_find_index;
// #endif
/**
 * @description:
 * @author: your name
 * @param {timer} *t
 * @return {*}
 */
static uint_fast8_t hx711s_event(struct timer *t) {
  sched_wake_task(&hx711s_wake);

  struct Hx711s *h = container_of(t, struct Hx711s, hx711s_timer);

  h->hx711s_timer.waketime += h->rest_ticks;

  if (h->flags & HX711S_START) {
    return SF_RESCHEDULE;
  }
  return SF_DONE;
}

/**
 * @description:
 * @author: your name
 * @param {uint32_t} *args
 * @return {*}
 */
void command_config_hx711s(uint32_t *args) {
  struct Hx711s *h = oid_alloc(args[0], command_config_hx711s, sizeof(*h));
  h->oid = args[0];
  h->hx711_count = args[1] & 0x0F;
  h->install_dir = (args[1] >> 4) & 0x0F;
  if (h->hx711_count > 4) shutdown("Max of 4 hx711");
  if (h->hx711_count < 1) shutdown("Min of 1 hx711");
  h->sg_mode = (args[3]&0x0F000000) >> 24;
  h->debug_data = (args[3]&0xF0000000) >> 28;
  struct gpio_in hx711_s0 = gpio_in_setup(HX717_S0, 0);
  struct gpio_in hx711_s1 = gpio_in_setup(HX717_S1, 0);
  h->sg_mode =((gpio_in_read(hx711_s0) > 0) ? 1 : 0) | (((gpio_in_read(hx711_s1) > 0) ? 1 : 0) << 1);
  h->find_index_mode = ((args[2] & 0xF000) >> 12);
  UART1_Printf("GPIO read sg_mode:%d\r\n", h->sg_mode);
  if (h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) {
    h->sample_period = 1500;
    // h->max_data_num = MAX_DATA_NUM;
    h->max_data_num = 16;
    h->find_index_mode = 0;
    h->min_th = 0;
  } else {
   h->sg_mode = SG_MODE_HX711_FULL_BRIDGE;
    h->sample_period = args[3]&0xFFFFFF;
    h->max_data_num = 12;
  }

  h->rest_ticks = CONFIG_CLOCK_FREQ / 1000000 * h->sample_period;
  ori_rest_ticks = h->rest_ticks;
  h->enable_channels = (args[2] & 0xFF);
  h->enable_hpf = ((args[2] & 0x0F00) >> 8);
  h->enable_shake_filter = ((args[2] & 0xF0000) >> 16);
  h->heartbeat_period = 2000000 / h->sample_period;
  h->kalman_q[0] =
      args[4];  // kalman_q[0] 临时作为应变片位接时 adc采样的阈值   abs(10000)
  h->kalman_r[0] =
      args[5];  // kalman_r[0] 临时作为应变片shake_motor时的振幅值 abs(2000)
  h->max_th = args[6];
  h->min_th = args[7];
  h->th_k = (args[8] & 0xFF);
  // h->bias_slope = ((args[8] & 0xFF00) >> 8);
  // h->k_slope = ((args[8] & 0xFFFF0000) >> 16);
  bias_slope = ((args[8] & 0xFF00) >> 8);
  k_slope = ((args[8] & 0xFFFF0000) >> 16);
  h->flags = 0;
  h->hx711s_timer.func = hx711s_event;
  sendf("debug_hx711s oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u",
        (int)args[0], (int)args[0], (int)args[1], (int)args[2], (int)args[3]);

  UART1_Printf(
      "[CMD]config_hx711s: oid: %d count: %d dir: %x "
      "chan: %d hpf:%d f_i_mode:%d sg_mode:%d "
      "shake_fil:%d ticks:%d hb_period:%d "
      "rst_peroid:%d"
      " k_q:%d k_r:%d max_th:%d min_th:%d th_k:%d rslope:%d m_slope:%d debug:%d\r\n",
      h->oid, h->hx711_count, (uint32_t)(h->install_dir), h->enable_channels,
      h->enable_hpf, h->find_index_mode,h->sg_mode, h->enable_shake_filter, h->rest_ticks,h->heartbeat_period,
      h->sample_period, h->kalman_q[0], h->kalman_r[0], h->max_th, h->min_th, h->th_k, bias_slope, k_slope,h->debug_data);

  h->is_calibration = 0;
  h->init_values[0] = 0;
  h->init_values[1] = 0;
  h->init_values[2] = 0;
  h->init_values[3] = 0;

  h->sample_values[0] = 0;
  h->sample_values[1] = 0;
  h->sample_values[2] = 0;
  h->sample_values[3] = 0;

  memcpy(&hx711s, h, sizeof(*h));
  
  // 默认使用通道2
  if (h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) {
    HX717_init(4);
  } else if (h->sg_mode == SG_MODE_HX717_FULL_BRIDGE) { 
    HX717_init(0);
    HX717_init(1);
    HX717_init(2);
    HX717_init(3);
  } else {

  }
  // 采样周期： h->sample_period
  high_pass_filter_init(5, (1000000 / h->sample_period) / h->hx711_count, 0);
  fusion_high_pass_filter_init(5, (1000000 / h->sample_period), 0);
}
DECL_COMMAND(
    command_config_hx711s,
    "config_hx711s oid=%c hx711_count=%c channels=%u rest_ticks=%u kalman_q=%u "
    "kalman_r=%u max_th=%u min_th=%u k=%u");

/**
 * @description:
 * @author: your name
 * @param {uint32_t} *args
 * @return {*}
 */
void command_add_hx711s(uint32_t *args) {
  uint8_t oid = args[0];
  uint8_t index = args[1];
  struct Hx711s *h = oid_lookup(oid, command_config_hx711s);
  if (index >= h->hx711_count) shutdown("Set hx711 past maximum count");
 
  if (h->sg_mode == SG_MODE_HX711_FULL_BRIDGE || h->sg_mode == SG_MODE_HX711_HALF_BRIDGE) {
    h->clks[index] = gpio_out_setup(args[2], 0);
    h->sdos[index] = gpio_in_setup(args[3], 0);
    UART1_Printf("init %d gpio\r\n ", index);
  }
  UART1_Printf("%d-clk_pin%d\r\n ", index,args[2]);

  sendf("debug_hx711s oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u",
        (int)args[0], (int)args[0], (int)args[1], (int)args[2], (int)args[3]);

  memcpy(&hx711s, h, sizeof(*h));
  UART1_Printf(
      "[CMD]command_add_hx711s oid=%c index=%c clk_pin=%u sdo_pin=%u\r\n ",
      (int)args[0], (int)args[1],(int)args[2], (int)args[3]);
}
DECL_COMMAND(command_add_hx711s,
             "add_hx711s oid=%c index=%c clk_pin=%u sdo_pin=%u");

/**
 * @description: 开始采集应变片滤波后的数据
 * @author: your name
 * @param {uint32_t} *args
 * @return {*}
 */
void command_query_hx711s(uint32_t *args) {
  uint8_t oid = args[0];
  struct Hx711s *h = oid_lookup(oid, command_config_hx711s);
  h->times_read = args[1];
  h->flags |= HX711S_START;
  sched_del_timer(&h->hx711s_timer);
  irq_disable();
  h->hx711s_timer.waketime = timer_read_time() + h->rest_ticks;
  sched_add_timer(&h->hx711s_timer);
  irq_enable();
  UART1_Printf("[CMD]command_query_hx711s, times_read: %d\r\n", h->times_read);
  h->traceflag = 1;
}
DECL_COMMAND(command_query_hx711s, "query_hx711s oid=%c times_read=%hu");

static int32_t position2point_k_index(int32_t x, int32_t y) {
  int32_t index = 0;
  switch (x) {
    case 10:
      switch (y)
      {
        case 10:
          index = 0;
          return index;

        case 57:
          index = 1;
          return index;

        case 104:
          index = 2;
          return index;

        case 151:
          index = 3;
          return index;

        case 198:
          index = 4;
          return index;

        case 246:
          index = 5;
          return index;
        
        default:
          return index;
      }

    case 57:
      switch (y)
      {
        case 10:
          index = 6;
          return index;

        case 57:
          index = 7;
          return index;

        case 104:
          index = 8;
          return index;

        case 151:
          index = 9;
          return index;

        case 198:
          index = 10;
          return index;

        case 246:
          index = 11;
          return index;
        
        default:
          return index;
      }
    case 104:
      switch (y)
      {
        case 10:
          index = 12;
          return index;

        case 57:
          index = 13;
          return index;

        case 104:
          index = 14;
          return index;
        case 151:
          index = 15;
          return index;

        case 198:
          index = 16;
          return index;

        case 246:
          index = 17;
          return index;
        
        default:
          return index;
      }
    case 151:
      switch (y)
      {
        case 10:
          index = 18;
          return index;

        case 57:
          index = 19;
          return index;

        case 104:
          index = 20;
          return index;

        case 151:
          index = 21;
          return index;

        case 198:
          index = 22;
          return index;

        case 246:
          index = 23;
          return index;
        
        default:
          return index;
      }
    case 198:
      switch (y)
      {
        case 10:
          index = 24;
          return index;

        case 57:
          index = 25;
          return index;

        case 104:
          index = 26;
          return index;

        case 151:
          index = 27;
          return index;

        case 198:
          index = 28;
          return index;

        case 246:
          index = 29;
          return index;
        
        default:
          return index;
      }
    case 246:
      switch (y)
      {
        case 10:
          index = 30;
          return index;

        case 57:
          index = 31;
          return index;

        case 104:
          index = 32;
          return index;

        case 151:
          index = 33;
          return index;

        case 198:
          index = 34;
          return index;

        case 246:
          index = 35;
          return index;
        
        default:
          return index;
      }
    default:
      return index;
  }
  return index;

}

/**
 * @description: 探针检测，上位机下发当前探针坐标，检查是否触发
 * @author: your name
 * @param {uint32_t} *args
 * probe_check_cmd: 1：开始检测 2:停止检测  3：标定检测
 * @return {*}
 */
void command_sg_probe_check(uint32_t *args) {
  static int print_index = 0;
  static int last_probe_check_cmd =0;
  uint8_t oid = args[0];
  struct Hx711s *h = oid_lookup(oid, command_config_hx711s);
  h->x = args[1];
  h->y = args[2];
  h->z = args[3];
  h->probe_check_cmd = args[4];
  sched_del_timer(&h->hx711s_timer);

  // 首次上电，z轴抬升，底部检测，不启用抖动检测和滤波
  if (h->x == 0 && h->y == 0 && h->z == 0 && h->probe_check_cmd == PROBE_CHECK_MODE_START) {
    h->is_bottom_detection = 1;
  } else {
    h->is_bottom_detection = 0;
  }

  // clear data
  h->is_trigger = 0;
  h->trigger_tick = 0;
  h->is_running_check = 0;
  h->now_trigger = 0;
  h->trigger_index = 0;


  h->sample_values[0] = 0;
  h->sample_values[1] = 0;
  h->sample_values[2] = 0;
  h->sample_values[3] = 0;



  for (int i = 0; i < MAX_SENSOR_NUM; i++) {
    for (int j = 0; j < MAX_DATA_NUM; j++) {
      data_list[i][j] = 0;
      filter_data_list[i][j] = 0;
      timestamp_list[i][j] = 0;
      data_list_f[i][j] = 0;
    }
  }

  // 停止
  if (h->probe_check_cmd == PROBE_CHECK_MODE_STOP) {
    h->flags &= ~HX711S_START;
    h->rest_ticks *= 1000;
    h->is_running_check &= ~HX711S_START;
    h->hx711s_timer.waketime = timer_read_time() + h->rest_ticks;
  }
  // 开始
  else if (h->probe_check_cmd == PROBE_CHECK_MODE_START || h->probe_check_cmd == PROBE_CHECK_MODE_CALIBRATION){
    static int32_t last_x=0, last_y=0; 
    current_point_index = position2point_k_index(h->x,h->y);
    if (last_x != h->x || last_y != h->y) {
      current_point_index_count = 0;
    }else {
      current_point_index_count++;
      if (current_point_index_count>3) {
        current_point_index_count = 0;
      }
    }
    last_x = h->x;
    last_y = h->y;
    
    h->rest_ticks = ori_rest_ticks;
    h->flags |= HX711S_START;
    h->is_running_check |= HX711S_START;
    sched_del_timer(&h->hx711s_timer);
    irq_disable();
    h->hx711s_timer.waketime = timer_read_time() + h->rest_ticks;
    sched_add_timer(&h->hx711s_timer);
    irq_enable();

    if (h->debug_data == 2) {
      sample_x = h->x;
      sample_y = h->y;
      print_index = 1;
      debug_data_count = 0;
    }
  }

  if (h->debug_data != 2) {
    UART1_Printf("[CMD]command_sg_probe_check, x:%d y:%d z:%d cmd:%d\r\n", h->x,
                h->y, h->z, h->probe_check_cmd);
  }
  // h->traceflag |= 0x02;
  // h->traceflag |= 0x04;
  // arg[1]: cmd 主从模式，回复上位机的命令
  sendf("debug_hx711s oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u", oid,
        (uint32_t)args[0], (uint32_t)h->probe_check_cmd, (uint32_t)args[2],
        (uint32_t)args[3]);
        
  if (h->probe_check_cmd == PROBE_CHECK_MODE_STOP && h->probe_check_cmd!=last_probe_check_cmd && h->debug_data) {
    for (int i=0; i<37; i++ ) { 
        UART1_Printf("k:%d %d %d %d f: %d %d %d %d\r\n", all_point_k[i][0],all_point_k[i][1],all_point_k[i][2],all_point_k[i][3],all_point_k_fix_index[i][0],all_point_k_fix_index[i][1],all_point_k_fix_index[i][2],all_point_k_fix_index[i][3]);
    }
  }
  last_probe_check_cmd = h->probe_check_cmd;

  if (h->debug_data == 2) {
    if (h->probe_check_cmd == PROBE_CHECK_MODE_STOP) {
      if (print_index < 10) {
        UART1_Printf("x:%d y:%d\r\n", sample_x,sample_y);
        // UART1_Printf("\r\n[DEBUG] data_num:%d trigger_index:%d find_index:%d\r\n",debug_data_count, debug_trigger_index, debug_find_index);
        UART1_Printf("%d %d %d %d\r\n",debug_data_count, debug_trigger_index, debug_trigger_data_value,debug_find_index);
      }
      
      for (; print_index < debug_data_count; print_index++) {
          UART1_Printf("%d %d\r\n", print_index, debug_fusion_data[print_index]);
        if ((print_index % 200 == 0)) {
          print_index++;
          UART1_Printf("%d %d\r\n", print_index, debug_fusion_data[print_index]);
          print_index++;
          break;
        }
      }
    }
  }
}
DECL_COMMAND(command_sg_probe_check,
             "sg_probe_check oid=%c x=%i y=%i z=%i cmd=%i");


/**
 * @description: 采集单通道
 * @author: your name
 * @param {uint8_t} choosed_sensor_hex: bit:
 * 7 6 5 4     3       2      1      0
 *          sensor4 sensor3 sensor2 sensor1
 * @return {*}
 */
static uint8_t update_hx711_data_from_exi(struct Hx711s *h) {
  uint8_t ret = 0;
  uint64_t tmp_tick = g_hx711_exi_tick[0];
  // 舍弃：±400 h->th_k
  int32_t bits = h->th_k;
  if (bits < 1) {
    h->th_k = 1;
  }
  for (int j = 0; j < h->hx711_count; j++) {
    g_hx711_exi_data[j] = g_hx711_exi_data[j] -  (g_hx711_exi_data[j]%bits);
    if ( h->sample_values[j] != g_hx711_exi_data[j])
    {
      h->time_stamp[j] = g_hx711_exi_tick[j]&0xFFFFFFFF;
      h->sample_values[j] = g_hx711_exi_data[j];
      if (tmp_tick < g_hx711_exi_tick[j]) {
         tmp_tick = g_hx711_exi_tick[j];
      }
      ret |= (1<<j);
    }
  }
  h->time_stamp[4] = tmp_tick & 0xFFFFFFFF;
  return ret;
}
/**
 * @description: 采集单通道
 * @author: your name
 * @param {uint8_t} choosed_sensor_hex: bit:
 * 7 6 5 4     3       2      1      0
 *          sensor4 sensor3 sensor2 sensor1
 * @return {*}
 */
static uint8_t hx711s_get_one(struct Hx711s *h, uint8_t choosed_sensor) {
  // uint32_t now_rtc_tick = rtc_getsec();
  static int32_t loop = 0;
  loop++;

  uint8_t is_data_valid = 1;
  // 查询A/D转换器是否就绪: 低电平表示数据就绪
  is_data_valid = ((gpio_in_read(h->sdos[choosed_sensor]) > 0) ? 1 : 0);
  for (int i=0; i<3000; i++) { 
    if (is_data_valid){ 
      delay_nus(1); 
      is_data_valid = ((gpio_in_read(h->sdos[choosed_sensor]) > 0) ? 1 : 0); 
    }else { 
      break; 
    } 
  }
  irq_disable();
  if (is_data_valid == 0) {
    h->sample_values[choosed_sensor] = 0;
    // 置低SCK
    gpio_out_write(h->clks[choosed_sensor], 0);
    uint8_t i = 0;
    for (i = 0; i < 24; i++) {
      // SCK上升沿
      gpio_out_write(h->clks[choosed_sensor], 1);
      delay_nus(1);
      // 数据位初始化为1
      h->sample_values[choosed_sensor] =
          (h->sample_values[choosed_sensor] << 1);
      // 置低SCK
      gpio_out_write(h->clks[choosed_sensor], 0);
      delay_nus(1);
      // 下降沿读取数据位
      uint8_t tmp = (gpio_in_read(h->sdos[choosed_sensor]) > 0 ? 1 : 0);
      // UART1_Printf("%d", tmp);
      h->sample_values[choosed_sensor] += tmp;
    }
    // UART1_Printf("\r\n");
    // 默认开启下一次转换：若需要修改通道或者增益，需要再发2-4个数据
    gpio_out_write(h->clks[choosed_sensor], 1);
    // 符号位转换
    h->sample_values[choosed_sensor] |=
        ((h->sample_values[choosed_sensor] & 0x00800000) != 0 ? 0xFF000000 : 0);
    // 置0
    gpio_out_write(h->clks[choosed_sensor], 0);

    // h->time_stamp[choosed_sensor] = now_tick;
    // if (h->traceflag & 0x04 && loop % 50 == 0) {
    //   UART1_Printf("[read] %d %d %d %d %d %d %d\r\n", loop, choosed_sensor,
    //                h->sample_values[0], h->sample_values[1],
    //                h->sample_values[2], h->sample_values[3], now_rtc_tick);
    // }
    irq_enable();
    return 1;
  } else {
    if (h->traceflag) {
      UART1_Printf("[WARNING]%d %d not ready.\r\n", loop, choosed_sensor);
    }
    irq_enable();
    return 0;
  }
  
  irq_enable();
  return 1;
}

/**
 * @description: 初始上电，标定应变片初始采集值
 * @author: your name
 * @param {Hx711s} *h
 * @return {*}
 */
static bool calibration_strain_gauge(struct Hx711s *h, uint8_t choosed_sensor) {
  static int32_t sample_num[4] = {0};
  static int32_t sample_values[4][1000] = {0};
  static int32_t sensor_min[4] = {0};
  static int32_t sensor_max[4] = {0};
 
  if (h->times_read == 0) {
    h->is_calibration = 0x80;
    for (int j = 0; j < h->hx711_count; j++) {
      if (sample_num[j] > 0) {
        h->amplitude_values[j] = sensor_max[j] - sensor_min[j];
        h->init_values[j] =
            median_filter(sample_values[j], sample_num[j]);  //中值滤波

        if (util_abs(h->init_values[j]) < util_abs(h->kalman_q[0])) {
          h->is_calibration |= (0x01 << j);
          // UART1_Printf(
          //     "[WARNING]  %d calibration failed, abs(init_value):%d < %d\r\n",
          //     j, util_abs(h->init_values[j]), util_abs(h->kalman_q[0]));
        }

        if (h->amplitude_values[j] < util_abs(h->kalman_r[0])) {
          h->is_calibration |= (0x01 << j);
          // UART1_Printf(
          //     "[WARNING]  %d calibration failed, amplitude_value:%d < %d\r\n",
          //     j, h->amplitude_values[j], util_abs(h->kalman_r[0]));
        }
      } else {
        UART1_Printf("[ERROR]  %d calibration failed\r\n", j);
        h->is_calibration = 0;
        return FALSE;
      }
      sample_num[j] = 0;
    }
    UART1_Printf(
        "[INFO] calibration times_read: %d sensor init_value: %d %d %d %d "
        "amplitude_value: %d %d %d %d\r\n",
        h->times_read, h->init_values[0], h->init_values[1], h->init_values[2],
        h->init_values[3], h->amplitude_values[0], h->amplitude_values[1],
        h->amplitude_values[2], h->amplitude_values[3]);
    return TRUE;
  }
  uint8_t update_data_ret = 0;
  if (h->sg_mode == SG_MODE_HX717_FULL_BRIDGE || h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) {
    update_data_ret = update_hx711_data_from_exi(h);
  } else {
    update_data_ret = hx711s_get_one(h,choosed_sensor);
  }
  if (!update_data_ret) {
    if (h->traceflag) 
    {
      UART1_Printf("[WARNING]update_hx711_data failed\r\n");
    }
  } else {
    
    if (h->sg_mode == SG_MODE_HX717_FULL_BRIDGE || h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) {
      h->times_read--;
      for (int j = 0; j < h->hx711_count; j++) {
        if (sensor_min[j] == 0) {
          sensor_min[j] = h->sample_values[j];
        }
        if (sensor_max[j] == 0) {
          sensor_max[j] = h->sample_values[j];
        }
        sample_values[j][sample_num[j]] =
            h->sample_values[j];
        if (h->sample_values[j] < sensor_min[j]) {
          sensor_min[j] = h->sample_values[j];
        }
        if (h->sample_values[j] > sensor_max[j]) {
          sensor_max[j] = h->sample_values[j];
        }

        if (h->times_read % 20 == 0 && h->debug_data) {
          UART1_Printf("[INFO] hx711: %d value: %d sample:%d sum_num: %d\r\n",
                      j, h->sample_values[j],
                      sample_values[j][sample_num[j]],
                      sample_num[j]);
        }
        sample_num[j]++;
      }
    }else {
      if ((choosed_sensor + 1) == h->hx711_count) { 
        h->times_read--; 
      }
      if (sensor_min[choosed_sensor] == 0) { 
        sensor_min[choosed_sensor] = h->sample_values[choosed_sensor]; 
      } 
      if (sensor_max[choosed_sensor] == 0) { 
        sensor_max[choosed_sensor] = h->sample_values[choosed_sensor]; 
      } 
      sample_values[choosed_sensor][sample_num[choosed_sensor]] = 
          h->sample_values[choosed_sensor]; 
      if (h->sample_values[choosed_sensor] < sensor_min[choosed_sensor]) { 
        sensor_min[choosed_sensor] = h->sample_values[choosed_sensor]; 
      } 
      if (h->sample_values[choosed_sensor] > sensor_max[choosed_sensor]) { 
        sensor_max[choosed_sensor] = h->sample_values[choosed_sensor]; 
      } 
  
      if (h->times_read % 10 == 0) { 
        UART1_Printf("[INFO] hx711: %d value: %d sample:%d sum_num: %d\r\n", 
                    choosed_sensor, h->sample_values[choosed_sensor], 
                    sample_values[choosed_sensor][sample_num[choosed_sensor]], 
                    sample_num[choosed_sensor]); 
      } 
      sample_num[choosed_sensor]++; 
    }
  }
  return TRUE;
}

/**
 * @description: 标定应变片，静置状态下采集应变片的初始值
 * @author: your name
 * @param {uint32_t} *args
 * @return {*}
 */
void command_calibration_sample(uint32_t *args) {
  uint8_t oid = args[0];
  struct Hx711s *h = oid_lookup(oid, command_config_hx711s);
  h->times_read = args[1];

  if (h->times_read < 1) {
    h->times_read = 50;
  } else if (h->times_read > 1000) {
    h->times_read = 1000;
  }
  h->rest_ticks = ori_rest_ticks;
  UART1_Printf("[CMD]command_calibration_sample, times_read:%d\r\n",
               h->times_read);
  h->is_calibration = 0;
  h->init_values[0] = 0;
  h->init_values[1] = 0;
  h->init_values[2] = 0;
  h->init_values[3] = 0;

  h->calibration_values[0] = 0;
  h->calibration_values[1] = 0;
  h->calibration_values[2] = 0;
  h->calibration_values[3] = 0;

  h->sample_values[0] = 0;
  h->sample_values[1] = 0;
  h->sample_values[2] = 0;
  h->sample_values[3] = 0;

  for (int i = 0; i < MAX_SENSOR_NUM; i++) {
    for (int j = 0; j < MAX_DATA_NUM; j++) {
      data_list[i][j] = 0;
      timestamp_list[i][j] = 0;
      data_list_f[i][j] = 0;
    }
  }

  // clear data
  h->is_trigger = 0;
  h->trigger_tick = 0;
  h->is_running_check = 0;
  h->probe_check_cmd = 0;

  // 开启任务
  h->flags |= HX711S_START;
  sched_del_timer(&h->hx711s_timer);
  irq_disable();
  h->hx711s_timer.waketime = timer_read_time() + h->rest_ticks;
  sched_add_timer(&h->hx711s_timer);
  // h->traceflag |= 0x04;
  irq_enable();
  memcpy(&hx711s, h, sizeof(*h));
  UART1_Printf("[INFO]start calibration sample task...\r\n");
  // 回复开启标定命令
  sendf("debug_hx711s oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u", oid, 1,
        (uint32_t)h->probe_check_cmd, (uint32_t)args[2], (uint32_t)args[3]);
}

DECL_COMMAND(command_calibration_sample,
             "calibration_sample oid=%c times_read=%hu");

/**
 * @description: 滤波融合
 * @author: your name
 * @param {uint8_t} choosed_sensor_hex: bit:
 * 7 6 5 4     3       2      1      0
 *          sensor4 sensor3 sensor2 sensor1
 * @return {*}
 */
static uint8_t hx711s_fusion_filter(struct Hx711s *h, uint8_t choosed_sensor,
                                    uint8_t is_fifo, uint8_t enable_hpf) {
  static int32_t loop = 0;
  static int32_t last_calibration_values[4] = {0};
  loop++;

  if (h->is_calibration) {
    for (int i=0; i<h->hx711_count;i++) {
      // 正相关，则添加 负号
      if ((h->install_dir >> i) & 0x01) {
        h->calibration_values[i] =
            -h->sample_values[i] + h->init_values[i];
      } else {
        h->calibration_values[i] =
            h->sample_values[i] - h->init_values[i];
      }
    }

    if (h->sg_mode != SG_MODE_HX717_HALF_BRIDGE ) {
      h->time_stamp[4] = h->time_stamp[choosed_sensor];
        // 异常值滤波+滑动窗口+高通滤波，保持最新的 h->max_data_num 组数据 
      if (!sliding_window_avg_exception_filter( 
              choosed_sensor, h->max_data_num, h->max_data_num-2, 
              &(h->calibration_values[choosed_sensor]), 
              h->time_stamp[choosed_sensor], 600000, is_fifo, enable_hpf)) { 
        // UART1_Printf("[WARNING]%d %d: exception-value:%d\r\n", loop, 
        //              choosed_sensor, h->calibration_values[choosed_sensor]); 
        return 0; 
      }
    }
    // 融合值为绝对值相加,
    // 不同点位触碰到热床时，三个应变片收到压力不一致，但值一定增大，使用求和平均融合方法
    h->fusion_filter_value = 0;
    int32_t tmp = 0;
    for (int i = 0; i < h->hx711_count; i++) {
      h->fusion_filter_value += h->calibration_values[i];
    }
    // 异常值滤波+滑动窗口+高通滤波，保持最新的 h->max_data_num 组数据
    if (!sliding_window_avg_exception_filter(
            4, h->max_data_num, h->max_data_num-2, &h->fusion_filter_value,
            h->time_stamp[4], 600000, is_fifo, enable_hpf)) {
      // UART1_Printf("[WARNING]%d: exception-value:%d\r\n", 4,
      //              h->calibration_values[choosed_sensor]);
      return 0;
    } else {
      if (h->traceflag & 0x02) {
        // if ((loop + choosed_sensor) % 100 == 0) {
        // 打印滑动窗口值
        // UART1_Printf(
        //     "[DEBUG]S: %d %d %d %d %d %d %d %d %d\r\n", choosed_sensor,
        //     data_list[choosed_sensor][0],
        //     data_list[choosed_sensor][1],
        //     data_list[choosed_sensor][2],
        //     data_list[choosed_sensor][3],
        //     data_list[choosed_sensor][4],
        //     data_list[choosed_sensor][5],
        //     data_list[choosed_sensor][6],
        //     data_list[choosed_sensor][7]);

        // 打印滑动窗口值
        UART1_Printf("[DEBUG]ST: %d %d %d %d %d %d %d %d %d\r\n",
                      choosed_sensor, timestamp_list[choosed_sensor][0],
                      timestamp_list[choosed_sensor][1],
                      timestamp_list[choosed_sensor][2],
                      timestamp_list[choosed_sensor][3],
                      timestamp_list[choosed_sensor][4],
                      timestamp_list[choosed_sensor][5],
                      timestamp_list[choosed_sensor][6],
                      timestamp_list[choosed_sensor][7]);

        // UART1_Printf("[fusion filter] %d %d %d %d %d %d %d\r\n", loop,
        //              choosed_sensor, h->calibration_values[0],
        //              h->calibration_values[1], h->calibration_values[2],
        //              h->calibration_values[3], h->fusion_filter_value);
        // 打印融合后的滑动窗口值
        // UART1_Printf("[DEBUG]F: %d %d %d %d %d %d %d %d %d\r\n", 4,
        //              data_list[4][0], data_list[4][1],
        //              data_list[4][2], data_list[4][3],
        //              data_list[4][4], data_list[4][5],
        //              data_list[4][6], data_list[4][7]);
        UART1_Printf("[DEBUG]FT: %d %d %d %d %d %d %d %d %d\r\n", 4,
                      timestamp_list[4][0], timestamp_list[4][1],
                      timestamp_list[4][2], timestamp_list[4][3],
                      timestamp_list[4][4], timestamp_list[4][5],
                      timestamp_list[4][6], timestamp_list[4][7]);
      }
      return 1;
    }
  } else {
    if (h->traceflag) {
      UART1_Printf("[WARNING]%d %d not calibration. %d\r\n", loop,
                   choosed_sensor);
    }
    return 0;
  }
  return 0;
}

/**
 * @description:
 * 针对触发完成后，获取的测量曲线，需要确定曲线中的哪一个点的压力值，对应的 Z
 * 轴为最终测量结果
 * @author: your name
 * @param {int} *array
 * @return {*}
 */
static int32_t find_trigger_index_new(int *array,struct Hx711s *h) {
  double val[MAX_DATA_NUM] = {};
  double val_transfer[MAX_DATA_NUM] = {};
  static int32_t last_current_point_index_count = 0; 
  // 1、保证数据方向为从小到大
  if (array[h->max_data_num - 1] - array[0] < 0) {
    for (int i = 0; i < h->max_data_num; i++) {
      val[i] = array[i] * -1.f;
    }
  } else {
    for (int i = 0; i < h->max_data_num; i++) {
      val[i] = array[i];
    }
  }
  // 线性回归找点
  // 2、对数据进行归一化，方便处理
  double val_min = val[h->max_data_num - 1], val_max = val[0], val_err = 0;
  for (int i = 0; i < h->max_data_num; i++) {
    val_min = val_min > val[i] ? val[i] : val_min;
    val_max = val_max < val[i] ? val[i] : val_max;
  }
  val_err = val_max - val_min;
  for (int i = 0; i < h->max_data_num; i++) {
    val_transfer[i] = (val[i] - val_min) / val_err;
  }

  // 4、计算并按给定角度翻转数据，以方便计算出最早的触发点
  double angle = atan((val_transfer[h->max_data_num - 1] - val_transfer[0]) /
                      (h->max_data_num - 1));
  double sinAngle = sin(-angle);
  double cosAngle = cos(-angle);
  //延原点(0,0)顺时针旋转angle度，这里可以不考虑X轴坐标值
  // UART1_Printf("val_transfer:");
  for (int i = 0; i < h->max_data_num; i++) {
    val_transfer[i] =
        ((i - 0) * sinAngle) + ((val_transfer[i] - 0) * cosAngle) + 0;
    // UART1_Printf("%d ", val_transfer[i]);
  }
  // UART1_Printf("\r\n");

  // 5、找出翻转后的最小值索引
  double min_val = val_transfer[0];
  // k值求解
  for (int i = 0; i < h->max_data_num; i++) {
    if (min_val > val_transfer[i]) {
      min_val = val_transfer[i];
      out_index = i;
    }
  }
  int32_t linear_out_index = out_index;
  // 线性回归 + 斜率 回退
  kk = (val[h->max_data_num-1] - val[out_index]) / (h->max_data_num - out_index);

  // k补偿计算方式：  0:线性拟合后计算  1:固定规律
  if (h->find_index_mode & 0x08) {
    fix_out_index = kk*k_slope/10000-bias_slope/10;
  } else {
    if (kk>2300) {
      fix_out_index = 11;
    }
    else if (kk>2200) {
      fix_out_index = 10;
    }
    else if (kk>2100) {
      fix_out_index = 9;
    }
    else if (kk>2000) {
      fix_out_index = 8;
    }
    else if (kk>1900) {
      fix_out_index = 7;
    }
    else if (kk>1700) {
      fix_out_index = 6;
    }
    else if (kk>1600) {
      fix_out_index = 5;
    }
    else if (kk>1500) {
      fix_out_index = 4;
    }
    else if (kk>1300) {
      fix_out_index = 3;
    }
    else if (kk>1000) {
      fix_out_index = 2;
    }
    else if (kk>900) {
      fix_out_index = 1;
    }
    else if (kk>800) {
      fix_out_index = 0;
    }
    else if (kk>700) {
      fix_out_index = -1;
    }
    else if (kk>600) {
      fix_out_index = -2;
    }
    else if (kk>500) {
      fix_out_index = -3;
    }
    else if (kk>400) {
      fix_out_index = -4;
    }
    else  {
      fix_out_index = -5;
    }
  }

  // 回退点的方式： 00:线性翻转找最低点回退 0后向阈值(从后往前)   10:前向阈值(从前往后) 
  if ((h->find_index_mode & 0x06) == 0x02) {
    for (int i = (h->max_data_num-1); i >=0; i--) {
      if (h->min_th > val[i]) {
        out_index = i;
        break;
      }
    }
    // UART1_Printf("th back: %d %d\r\n",out_index,find_index_mode);
  } else if ((h->find_index_mode & 0x06) == 0x04) {
    for (int i = 0; i < h->max_data_num ; i++) {
      if (h->min_th < val[i]) {
        out_index = i;
        break;
      }
    }
    // UART1_Printf("th front: %d %d\r\n",out_index,find_index_mode);
  } else {
    // UART1_Printf("th ？？: %d %d\r\n",out_index,find_index_mode);
    // 保持
  }


  // 应用k补偿:  0:不应用 1:应用
 if (h->find_index_mode & 0x01) { 
    out_index += fix_out_index;
    if (out_index <0 ) {
      out_index = 0;
    } else if (out_index > (h->max_data_num-1) ) {
      out_index = h->max_data_num-1;
    }
  } 
  else {
    // 保持
  }
  if (current_point_index_count != last_current_point_index_count) {
    all_point_k[current_point_index][current_point_index_count] = kk;
    all_point_k_fix_index[current_point_index][current_point_index_count] = fix_out_index;
    UART1_Printf("k:%d fix_index:%d  out_index:%d l_out_index:%d\r\n", kk, fix_out_index, out_index,linear_out_index);
  }
  last_current_point_index_count = current_point_index_count;

  return out_index;
}

/**
 * @description: 找出第一个触发的点
 * @author: your name
 * @param {int} *array
 * @param {int} data_num
 * @return {*}
 */
static int32_t find_trigger_index(int *array, int data_num, int th) {
  for (int i = (data_num - 1); i > 0; i--) {
    if (util_abs(array[i]) > th && util_abs(array[i - 1]) > th) {
      // UART1_Printf("[trigger index]%d \r\n", i);
      return i;
    }
  }
  return 0;
}

/**
 * @description:
 * @author: your name
 * @param {int} *data_list
 * @param {int32_t} min_th
 * @param {int32_t} max_th
 * @param {int32_t} enable_shake_filter: 使能手拍干扰、桌子振动干扰
 * @return {*}
 */
static uint8_t check_tregger(int *data_list,struct Hx711s *h) {
  // 标定模式检测触发：使用均值滤波判定
  if (h->probe_check_cmd == PROBE_CHECK_MODE_CALIBRATION) {
    int avg_data = 0;
    for(int i=0; i<h->max_data_num; i++) {
      avg_data += data_list[i];
    }
    avg_data = avg_data/h->max_data_num;
    if (avg_data < -h->min_th) {
      // UART1_Printf("avg_data:%d min_th:%d\r\n", avg_data,h->min_th);
      return 0x60;
    }
  }
  double val_p[MAX_DATA_NUM];

  //最高优先级，压力大于最大压力，则无条件停止。 采集数量>=MAX_DATA_NUM
  int trriger = 0;
  // if (h->sg_mode == SG_MODE_HX717_HALF_BRIDGE)
  // {
  //   trriger = data_list[h->max_data_num - 1] <= -h->max_th &&
  //             data_list[h->max_data_num - 2] <= -h->min_th &&
  //             data_list[h->max_data_num - 3] <= -h->min_th &&
  //             data_list[h->max_data_num - 4] <= -h->min_th;
  // } else 
  {
    trriger = data_list[h->max_data_num - 1] <= -h->max_th &&
              (util_abs(data_list[0]) > 0);
  }
  if (trriger)
  {
    // UART1_Printf("\r\nmax t: %d %d\r\n", data_list[h->max_data_num - 1],
    // max_th);
    if (h->enable_shake_filter) {
      int i = 0;
      if (h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) {
        i = h->max_data_num-10;
      } else {
        i = 0;
      }
      for (;i < h->max_data_num; i++) {
        if (data_list[i] > (h->min_th)) {
          // UART1_Printf(
          //     "shake noise..... last data: i:%d data:%d "
          //     "th:%d............\r\n",
          //     i, data_list[i], min_th / 10);
          return 0;
        }
    }
    }
    return 0x40;
  }

  //  1、检测未尾3个点大小的是否有序递增
  if (!((util_abs(data_list[h->max_data_num - 1]) >
         util_abs(data_list[h->max_data_num - 2])) &&
        (util_abs(data_list[h->max_data_num - 2]) >
         util_abs(data_list[h->max_data_num - 3])))) {
    // UART1_Printf("c1");
    return 0;
  }

  // 2. 检测未3个点绝对值是否大于其它任意点
  for (int i = 0; i < h->max_data_num - 3; i++) {
    if (util_abs(data_list[h->max_data_num - 1]) > data_list[i] ||
        util_abs(data_list[h->max_data_num - 2]) > data_list[i] ||
        util_abs(data_list[h->max_data_num - 3]) > data_list[i]) {
      // UART1_Printf("c2");
      return 0;
    }
  }

  // 3.1对数据进行归一化，方便处理
  double val_min = +0xFFFFFFFF, val_max = -0x00FFFFFF, val_avg = 0, val_err = 0;
  for (int i = 0; i < h->max_data_num; i++) {
    val_min = val_min > data_list[i] ? data_list[i] : val_min;
    val_max = val_max < data_list[i] ? data_list[i] : val_max;
    val_avg += data_list[i];
  }
  val_avg /= h->max_data_num;
  val_err = val_max - val_min;
  for (int i = 0; i < h->max_data_num; i++) {
    val_p[i] = (float)(((double)data_list[i] - val_min) / val_err);
  }

  // 3.2.保证所有点针对最后一个点的斜率均大于40度,可有郊防止过于灵敏而导致的误触发。
  for (int i = 0; i < h->max_data_num - 1; i++) {
    double k =
        (val_p[0] - val_p[i]) / ((h->max_data_num - i) * 1.0f / h->max_data_num);
    if (util_abs(k) < 0.8) {
      // UART1_Printf("c3");
      return 0;
    }
    if (h->enable_shake_filter) {
      if (k > 0.8) {
        // UART1_Printf("shake noise... k:%f..............", k);
        return 0;
      }
    }
  }
  // 4.限制最小值，防止误触发。
  if (util_abs(data_list[h->max_data_num - 1]) < h->min_th) {
    // UART1_Printf("c4");
    return 0;
  }

  // 正直为扰动数据
  if (h->enable_shake_filter) {
    if (data_list[h->max_data_num - 1] > h->min_th) {
      // UART1_Printf("shake noise.......last_data:%d..........",
      //              data_list[h->max_data_num - 1]);
      return 0;
    }
  }

  UART1_Printf("\r\nother trigger\r\n");
  return 0x20;
}

/**
 * @description: 触发检测
 * 1.最后3个点的值，均大于之前测量的所有点
 * 2.最后3个点的值，按从小到大排列
 * 3.最后1个点，相对于之前所有点的斜率，均大于约40度
 * 以上三个条件，基本上可以覆盖99%以上的触发场景，但仍然会有少量过触发（过于灵敏）、欠触发（过于不灵敏）、不触发（曲线变化较慢而被高通滤波器滤除）、错触发（Z 值错误）。这里分别会有对应的检测机制来应对。
过触发：设定最小触发阈值，最后1个点的值需要大于给定阈值。
欠触发：设定最大触发阈值，最后1个点的值如果大于给定阈值，则强制认为触发完成。
不触发：配合经典阈值测量方法为辅（即以往产品正在使用的测量方法）。
错触发：对最终测量结果，进行数据验证，并配合重复测量
 * @author: your name
 * @param {uint8_t} choosed_sensor_hex: bit:
 * 7 6 5 4     3       2      1      0
 *          sensor4 sensor3 sensor2 sensor1
 * @return {int32_t} 触发时返回: 低8位触发模式，高8位： trigger index
 */
static int32_t trigger_check_new(struct Hx711s *h, uint8_t choosed_sensor) {
  int32_t trigger = 0;
  uint8_t trigger_index = 0;
  uint8_t check_tregger_flag = 0;
  // 热床底部检测，关闭抖动滤波
  if (h->is_bottom_detection) {
    h->enable_shake_filter = 0;
  }
  // 优先融合数据触发
  if ((h->enable_channels >> 4) & 0x01) {
    check_tregger_flag = check_tregger(data_list[4], h);
    if (check_tregger_flag) {
      trigger_index = find_trigger_index_new(data_list[4], h);
      trigger |= 0x10;  // trigger sensor index
      trigger |= (check_tregger_flag & 0x60);
      trigger |= (trigger_index << 8);
      // UART1_Printf("[trigger f] %d %d %d %d %d %d %d %d %d index:%d\r\n", 4,
      //              data_list[4][0], data_list[4][1], data_list[4][2],
      //              data_list[4][3], data_list[4][4], data_list[4][5],
      //              data_list[4][6], data_list[4][7], trigger_index);
      // support Production testing fixtures
      trigger |= (0x01 << choosed_sensor);  // trigger sensor index
      return trigger;
    }
  }
  if ((h->enable_channels >> choosed_sensor) & 0x01) {
    check_tregger_flag = check_tregger(data_list[choosed_sensor], h);
    if (check_tregger_flag) {
      trigger_index = find_trigger_index_new(data_list[choosed_sensor],h);
      trigger |= (0x01 << choosed_sensor);  // trigger sensor index
      trigger |= (check_tregger_flag & 0x60);
      trigger |= trigger_index << 8;
      // UART1_Printf("[trigger] %d %d %d %d %d %d %d %d %d index:%d\r\n",
      //              choosed_sensor, data_list[choosed_sensor][0],
      //              data_list[choosed_sensor][1],
      //              data_list[choosed_sensor][2],
      //              data_list[choosed_sensor][3],
      //              data_list[choosed_sensor][4],
      //              data_list[choosed_sensor][5],
      //              data_list[choosed_sensor][6],
      //              data_list[choosed_sensor][7], trigger_index);

      return trigger;
    }
  }

  return 0;
}

/**
 * @description: 触发检测
 * @author: your name
 * @param {uint8_t} choosed_sensor_hex: bit:
 * 7 6 5 4     3       2      1      0
 *          sensor4 sensor3 sensor2 sensor1
 * @return {int32_t} 触发时返回: 低8位触发模式，高8位： trigger index
 */
static int32_t trigger_check(struct Hx711s *h, uint8_t choosed_sensor) {
  // 一次线性拟合
  float k0 = 0;
  float b0;
  static float k0_list_f[MAX_SENSOR_NUM][MAX_DATA_NUM] = {};
  static float b0_list_f[MAX_SENSOR_NUM][MAX_DATA_NUM] = {};

  int32_t trigger = 0;
  uint8_t trigger_index = 0;

  if ((h->enable_channels >> 4) & 0x01) {
    // 融合之后
    hx711_least_square(data_list[4], h->max_data_num, &k0, &b0, h->max_th);
    if (DEBUG_HX711 == 1) {
      array_sliding_window_f(k0_list_f[4], h->max_data_num, k0, 1);
      array_sliding_window_f(b0_list_f[4], h->max_data_num, b0, 1);
    }
    if (k0 > h->th_k) {
      if (DEBUG_HX711 == 1) {
        UART1_Printf("[fusion trigger] kkkkk:%f\r\n", k0);
        // 打印融合后的滑动窗口值
        UART1_Printf("[trigger F] %d %d %d %d %d %d %d %d %d\r\n", 4,
                     data_list[4][0], data_list[4][1], data_list[4][2],
                     data_list[4][3], data_list[4][4], data_list[4][5],
                     data_list[4][6], data_list[4][7]);
        UART1_Printf("[trigger Fk] %d %f %f %f %f %f %f %f %f\r\n", 4,
                     k0_list_f[4][0], k0_list_f[4][1], k0_list_f[4][2],
                     k0_list_f[4][3], k0_list_f[4][4], k0_list_f[4][5],
                     k0_list_f[4][6], k0_list_f[4][7]);
        // UART1_Printf("[trigger Fb] %d %f %f %f %f %f %f %f %f\r\n", 4,
        //              b0_list_f[4][0], b0_list_f[4][1], b0_list_f[4][2],
        //              b0_list_f[4][3], b0_list_f[4][4], b0_list_f[4][5],
        //              b0_list_f[4][6], b0_list_f[4][7]);

        UART1_Printf("[trigger Ft] %d %d %d %d %d %d %d %d %d\r\n", 4,
                     timestamp_list[4][0], timestamp_list[4][1],
                     timestamp_list[4][2], timestamp_list[4][3],
                     timestamp_list[4][4], timestamp_list[4][5],
                     timestamp_list[4][6], timestamp_list[4][7]);
      }
      trigger_index = find_trigger_index(data_list[4], h->max_data_num, h->min_th);
      if (trigger_index > 0) {
        trigger = (0x0F & (4 + 1));  // sensor index
        trigger |= (trigger_index << 8);
        return trigger;
      }
    }
  }
  if ((h->enable_channels >> choosed_sensor) & 0x01) {
    // 求解斜率
    hx711_least_square(data_list[choosed_sensor], h->max_data_num, &k0, &b0,
                       h->max_th);
    if (DEBUG_HX711 == 1) {
      array_sliding_window_f(k0_list_f[choosed_sensor], h->max_data_num, k0, 1);
      array_sliding_window_f(b0_list_f[choosed_sensor], h->max_data_num, b0, 1);
    }
    if (k0 > h->th_k) {
      if (DEBUG_HX711 == 1) {
        UART1_Printf("[trigger] %d kkkkk:%f\r\n", choosed_sensor, k0);
        // 打印滑动窗口值
        UART1_Printf("[trigger I] %d %d %d %d %d %d %d %d %d\r\n",
                     choosed_sensor, data_list[choosed_sensor][0],
                     data_list[choosed_sensor][1], data_list[choosed_sensor][2],
                     data_list[choosed_sensor][3], data_list[choosed_sensor][4],
                     data_list[choosed_sensor][5], data_list[choosed_sensor][6],
                     data_list[choosed_sensor][7]);
        UART1_Printf("[trigger Ik] %d %f %f %f %f %f %f %f %f\r\n",
                     choosed_sensor, k0_list_f[choosed_sensor][0],
                     k0_list_f[choosed_sensor][1], k0_list_f[choosed_sensor][2],
                     k0_list_f[choosed_sensor][3], k0_list_f[choosed_sensor][4],
                     k0_list_f[choosed_sensor][5], k0_list_f[6][6],
                     k0_list_f[choosed_sensor][7]);
        // UART1_Printf("[trigger Ib] %d %f %f %f %f %f %f %f %f\r\n",
        //              choosed_sensor, b0_list_f[choosed_sensor][0],
        //              b0_list_f[choosed_sensor][1],
        //              b0_list_f[choosed_sensor][2],
        //              b0_list_f[choosed_sensor][3],
        //              b0_list_f[choosed_sensor][4],
        //              b0_list_f[choosed_sensor][5],
        //              b0_list_f[choosed_sensor][6],
        //              b0_list_f[choosed_sensor][7]);
        UART1_Printf("[trigger It] %d %d %d %d %d %d %d %d %d\r\n",
                     choosed_sensor, timestamp_list[choosed_sensor][0],
                     timestamp_list[choosed_sensor][1],
                     timestamp_list[choosed_sensor][2],
                     timestamp_list[choosed_sensor][3],
                     timestamp_list[choosed_sensor][4],
                     timestamp_list[choosed_sensor][5],
                     timestamp_list[choosed_sensor][6],
                     timestamp_list[choosed_sensor][7]);
      }
      trigger_index = find_trigger_index(data_list[choosed_sensor],
                                         h->max_data_num, h->min_th);
      if (trigger_index > 0) {
        trigger = (0x0F & (choosed_sensor + 1));  // sensor index
        trigger |= trigger_index << 8;
        return trigger;
      } else {
        UART1_Printf("[find index] failed\r\n");
      }
    }
  }

  if ((h->enable_channels >> 4) & 0x01) {
    if (util_abs(data_list[4][0]) > h->max_th &&
        util_abs(data_list[4][1]) > h->max_th &&
        util_abs(data_list[4][2]) > h->max_th) {
      // if (DEBUG_HX711 == 1) {
      //   // // 打印滑动窗口值
      //   // UART1_Printf("[trigger F max_th] %d %d %d %d %d %d %d %d %d\r\n",
      //   4,
      //   //              data_list[4][0], data_list[4][1],
      //   data_list[4][2],
      //   //              data_list[4][3], data_list[4][4],
      //   data_list[4][5],
      //   //              data_list[4][6], data_list[4][7]);

      //   // UART1_Printf("[trigger Ft max_th] %d %d %d %d %d %d %d %d %d\r\n",
      //   4,
      //   //              timestamp_list[4][0], timestamp_list[4][1],
      //   //              timestamp_list[4][2], timestamp_list[4][3],
      //   //              timestamp_list[4][4], timestamp_list[4][5],
      //   //              timestamp_list[4][6],
      //   timestamp_list[4][7]);
      // }
      trigger_index = find_trigger_index(data_list[4], h->max_data_num, h->max_th);
      if (trigger_index > 0) {
        trigger = (0x0F & (4 + 1));  // sensor index
        trigger |= 0x80;
        trigger |= trigger_index << 8;
        return trigger;
      } else {
        UART1_Printf("[find index] failed\r\n");
      }
    }
  }

  if ((h->enable_channels >> choosed_sensor) & 0x01) {
    if (util_abs(data_list[choosed_sensor][0]) > h->max_th &&
        util_abs(data_list[choosed_sensor][1]) > h->max_th &&
        util_abs(data_list[choosed_sensor][2]) > h->max_th) {
      // // 打印滑动窗口值
      // UART1_Printf("[trigger I max_th] %d %d %d %d %d %d %d %d %d\r\n",
      //              choosed_sensor, data_list[choosed_sensor][0],
      //              data_list[choosed_sensor][1],
      //              data_list[choosed_sensor][2],
      //              data_list[choosed_sensor][3],
      //              data_list[choosed_sensor][4],
      //              data_list[choosed_sensor][5],
      //              data_list[choosed_sensor][6],
      //              data_list[choosed_sensor][7]);
      // UART1_Printf(
      //     "[trigger It max_th] %d %d %d %d %d %d %d %d %d\r\n",
      //     choosed_sensor, timestamp_list[choosed_sensor][0],
      //     timestamp_list[choosed_sensor][1],
      //     timestamp_list[choosed_sensor][2],
      //     timestamp_list[choosed_sensor][3],
      //     timestamp_list[choosed_sensor][4],
      //     timestamp_list[choosed_sensor][5],
      //     timestamp_list[choosed_sensor][6],
      //     timestamp_list[choosed_sensor][7]);

      trigger_index = find_trigger_index(data_list[choosed_sensor],
                                         h->max_data_num, h->max_th);
      if (trigger_index > 0) {
        trigger = (0x0F & (choosed_sensor + 1));  // sensor index
        trigger |= 0x80;
        trigger |= trigger_index << 8;
        return trigger;
      } else {
        UART1_Printf("[find index] failed\r\n");
      }
    }
  }

  return trigger;
}

/**
 * @description:
 * @author: your name
 * @return {*}
 */
void hx711s_task(void) {
  uint8_t oid;
  struct Hx711s *h;
  static uint8_t choosed_sensor = 4;
  static uint32_t loop = 0;
  static uint32_t last_rep_loop = 0;
  static uint8_t last_is_calibration = 0;
  static uint8_t last_is_trigger = 0;
  static uint8_t last_sendf_tick = 0;
  static uint32_t last_trigger_tick = 0;

  if (!sched_check_wake(&hx711s_wake)) {
    return;
  }
  foreach_oid(oid, h, command_config_hx711s) {
    if (!(h->flags & HX711S_START)) {
      return;
    }
    
    if (h->sg_mode != SG_MODE_HX717_HALF_BRIDGE && h->hx711_count == 4) {
      if (choosed_sensor == 0) { 
        choosed_sensor = 2; 
      } 
      else if (choosed_sensor == 1) { 
        choosed_sensor = 3; 
      } 
      else if (choosed_sensor == 2) { 
        choosed_sensor = 1; 
      } 
      else if (choosed_sensor == 3) { 
        choosed_sensor = 0; 
      } 
      // 1.选择sensor 
      if (choosed_sensor >= h->hx711_count) { 
        choosed_sensor = 0; 
      }
    }
    loop++;
    if (!(h->is_calibration & 0x80)) {
      last_is_calibration = h->is_calibration;
      calibration_strain_gauge(h, choosed_sensor);
    } else {
      // 2.采样
      uint32_t now_tick = timer_read_time();
      uint8_t update_data_ret = 0;
      if (h->sg_mode == SG_MODE_HX717_FULL_BRIDGE || h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) {
        update_data_ret = update_hx711_data_from_exi(h);
      }else {
        update_data_ret = hx711s_get_one(h,choosed_sensor);
        now_tick = timer_read_time();
        h->time_stamp[choosed_sensor] = now_tick;
      }
      if (!update_data_ret) {
        if (h->traceflag)
        {
          UART1_Printf("update_hx711_data_from_exi err %d\r\n", loop);
        }
      } else {
        static uint64_t last_tick = 0;
        now_tick += (now_tick < last_tick ? 0xFFFFFFFF : 0);
        uint64_t now_inter_ms =
            (now_tick - last_tick) * 1000.0f / CONFIG_CLOCK_FREQ;
        last_tick = now_tick; 
        // 3.安装方向转换+滤波+融合
        uint8_t trigger_sensor;
        if (!hx711s_fusion_filter(h, choosed_sensor, 0, h->enable_hpf)) {
          // UART1_Printf("hx711s_fusion_filter err %d\r\n", loop);
        } else {
          // 触发点求解: 触发标志+触发点
          int trigger = trigger_check_new(h, choosed_sensor);
          if (h->traceflag & 0x04 && loop % 50 == 0) {
            UART1_Printf("[read] %d %d %d %d %d %d %d\r\n", loop,
                         choosed_sensor, h->calibration_values[0],
                         h->calibration_values[1], h->calibration_values[2],
                         h->calibration_values[3], h->fusion_filter_value);
          }

          if (h->debug_data == 2) {
            debug_fusion_data[debug_data_count] = h->fusion_filter_value;
            debug_data_count++;
            if (debug_data_count >= DEBUG_DATA_NUM) {
              debug_data_count = 0;
            }
          }
          if (h->is_trigger <= 0) {
            h->is_trigger = trigger & 0xFF;
            if (trigger > 0) {
              h->now_trigger = h->is_trigger;
              h->trigger_index = (trigger >> 8) & 0xFF;
              if (h->is_trigger & 0x10) {
                trigger_sensor = 4;
                h->trigger_tick = timestamp_list[4][h->trigger_index];
              } else {
                trigger_sensor = choosed_sensor;
                h->trigger_tick = timestamp_list[choosed_sensor][h->trigger_index];
              }
            }
          }
        }
        // uint32_t rtc_tick = rtc_getsec();
        now_inter_ms =
            (now_tick - last_sendf_tick) * 1000.0f / CONFIG_CLOCK_FREQ;

        // heatbeat 上报 or is_trigger or calibration status changed
        if (loop % h->heartbeat_period == 0 
            || h->is_trigger > 0
            || h->now_trigger > 0
            || last_is_calibration != h->is_calibration) {
          // 回复命令： oid 是否标定 间隔 当前时间戳 是否触发： 1-触发 0-心跳
          // time_tick
          int resend = 80;
          // if (h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) {
          //   resend = 50;
          // }
          if (util_abs(loop - last_rep_loop) > resend || (last_is_trigger == 0)) {
            last_rep_loop = loop;
            // 触发时间不一样才上报，减少总线上报
            sendf("sg_resp oid=%c vd=%c it=%c nt=%u r=%u tt=%u",
                  (uint8_t)h->oid, (uint8_t)h->is_calibration,
                  (uint8_t)h->trigger_index, (uint32_t)h->trigger_tick,
                  (uint32_t)h->now_trigger, (uint32_t)last_tick); 

          if (h->debug_data == 2) {
            if (h->now_trigger) {
              debug_trigger_index = debug_data_count-1;
              debug_trigger_data_value = h->fusion_filter_value;
              debug_find_index = h->trigger_index;
            }
          } 
            // if (h->traceflag)
            {
              UART1_Printf(
                  "\nloop:%d choosed_sensor %d index:%d trigger:%d trigger_tick:%d \r\n",
                  loop, (uint32_t)choosed_sensor, (uint32_t)h->trigger_index, (uint32_t)h->now_trigger, h->trigger_tick);              

              // UART1_Printf(
              //     "%d %d %d %d %d %d %d %d %d\r\n", choosed_sensor,
              //     data_list[choosed_sensor][0], data_list[choosed_sensor][1],
              //     data_list[choosed_sensor][2], data_list[choosed_sensor][3],
              //     data_list[choosed_sensor][4], data_list[choosed_sensor][5],
              //     data_list[choosed_sensor][6],
              //     data_list[choosed_sensor][7]);

              if (h->sg_mode!=SG_MODE_HX717_HALF_BRIDGE && h->debug_data) {
                UART1_Printf("%d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d\r\n", 0,
                            data_list[0][0], data_list[0][1], data_list[0][2],
                            data_list[0][3], data_list[0][4], data_list[0][5],
                            data_list[0][6], data_list[0][7], data_list[0][8],
                            data_list[0][9], data_list[0][10], data_list[0][11],
                            data_list[0][12], data_list[0][13], data_list[0][14],
                            data_list[0][15], data_list[0][16], data_list[0][17],
                            data_list[0][18], data_list[0][19], data_list[0][20],
                            data_list[0][21], data_list[0][22], data_list[0][23],
                            data_list[0][24], data_list[0][25], data_list[0][26],
                            data_list[0][27], data_list[0][28], data_list[0][29]);
              }
              // UART1_Printf("ori: %d %d %d %d %d %d %d %d %d\r\n", 0,
              //              filter_data_list[0][0], filter_data_list[0][1],
              //              filter_data_list[0][2], filter_data_list[0][3],
              //              filter_data_list[0][4], filter_data_list[0][5],
              //              filter_data_list[0][6], filter_data_list[0][7]);
              if (h->hx711_count > 1 && h->sg_mode!=SG_MODE_HX717_HALF_BRIDGE && h->debug_data) {
                UART1_Printf("%d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d\r\n", 1,
                           data_list[1][0], data_list[1][1], data_list[1][2],
                           data_list[1][3], data_list[1][4], data_list[1][5],
                           data_list[1][6], data_list[1][7], data_list[1][8],
                           data_list[1][9], data_list[1][10], data_list[1][11],
                           data_list[1][12], data_list[1][13], data_list[1][14],
                           data_list[1][15], data_list[1][16], data_list[1][17],
                           data_list[1][18], data_list[1][19], data_list[1][20],
                           data_list[1][21], data_list[1][22], data_list[1][23],
                           data_list[1][24], data_list[1][25], data_list[1][26],
                           data_list[1][27], data_list[1][28], data_list[1][29]);
                // UART1_Printf("ori: %d %d %d %d %d %d %d %d %d\r\n", 1,
                //              filter_data_list[1][0], filter_data_list[1][1],
                //              filter_data_list[1][2], filter_data_list[1][3],
                //              filter_data_list[1][4], filter_data_list[1][5],
                //              filter_data_list[1][6], filter_data_list[1][7]);
              }

              if (h->hx711_count > 2 && h->sg_mode!=SG_MODE_HX717_HALF_BRIDGE  && h->debug_data) {
                UART1_Printf("%d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d\r\n", 2,
                           data_list[2][0], data_list[2][1], data_list[2][2],
                           data_list[2][3], data_list[2][4], data_list[2][5],
                           data_list[2][6], data_list[2][7], data_list[2][8],
                           data_list[2][9], data_list[2][10], data_list[2][11],
                           data_list[2][12], data_list[2][13], data_list[2][14],
                           data_list[2][15], data_list[2][16], data_list[2][17],
                           data_list[2][18], data_list[2][19], data_list[2][20],
                           data_list[2][21], data_list[2][22], data_list[2][23],
                           data_list[2][24], data_list[2][25], data_list[2][26],
                           data_list[2][27], data_list[2][28], data_list[2][29]);
                // UART1_Printf("ori: %d %d %d %d %d %d %d %d %d\r\n", 2,
                //              filter_data_list[2][0], filter_data_list[2][1],
                //              filter_data_list[2][2], filter_data_list[2][3],
                //              filter_data_list[2][4], filter_data_list[2][5],
                //              filter_data_list[2][6], filter_data_list[2][7]);
              }

              if (h->hx711_count > 3 && h->sg_mode!=SG_MODE_HX717_HALF_BRIDGE  && h->debug_data) {
                UART1_Printf("%d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d\r\n", 3,
                           data_list[3][0], data_list[3][1], data_list[3][2],
                           data_list[3][3], data_list[3][4], data_list[3][5],
                           data_list[3][6], data_list[3][7], data_list[3][8],
                           data_list[3][9], data_list[3][10], data_list[3][11],
                           data_list[3][12], data_list[3][13], data_list[3][14],
                           data_list[3][15], data_list[3][16], data_list[3][17],
                           data_list[3][18], data_list[3][19], data_list[3][20],
                           data_list[3][21], data_list[3][22], data_list[3][23],
                           data_list[3][24], data_list[3][25], data_list[3][26],
                           data_list[3][27], data_list[3][28], data_list[3][29]);
                // UART1_Printf("ori: %d %d %d %d %d %d %d %d %d\r\n", 3,
                //              filter_data_list[3][0], filter_data_list[3][1],
                //              filter_data_list[3][2], filter_data_list[3][3],
                //              filter_data_list[3][4], filter_data_list[3][5],
                //              filter_data_list[3][6], filter_data_list[3][7]);
              }
              if (h->debug_data) {
                UART1_Printf("%d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d %d %d %d\r\n", 4,
                            data_list[4][0], data_list[4][1], data_list[4][2],
                            data_list[4][3], data_list[4][4], data_list[4][5],
                            data_list[4][6], data_list[4][7], data_list[4][8],
                            data_list[4][9], data_list[4][10], data_list[4][11],
                            data_list[4][12], data_list[4][13], data_list[4][14],
                            data_list[4][15], data_list[4][16], data_list[4][17],
                            data_list[4][18], data_list[4][19], data_list[4][20],
                            data_list[4][21], data_list[4][22], data_list[4][23],
                            data_list[4][24], data_list[4][25], data_list[4][26],
                            data_list[4][27], data_list[4][28], data_list[4][29]);
              }
              // UART1_Printf("ori: %d %d %d %d %d %d %d %d %d\r\n", 4,
              //              filter_data_list[4][0], filter_data_list[4][1],
              //              filter_data_list[4][2], filter_data_list[4][3],
              //              filter_data_list[4][4], filter_data_list[4][5],
              //              filter_data_list[4][6], filter_data_list[4][7]);
              // UART1_Printf("t-4: %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
              //              timestamp_list[4][0], timestamp_list[4][1],
              //              timestamp_list[4][2], timestamp_list[4][3],
              //              timestamp_list[4][4], timestamp_list[4][5],
              //              timestamp_list[4][6], timestamp_list[4][7],
              //              timestamp_list[4][8], timestamp_list[4][9], 
              //              timestamp_list[4][10], timestamp_list[4][11]);

              // UART1_Printf("%d %d %d %d %d %d %d %d %d\r\n", 1,
              //              timestamp_list[1][0], timestamp_list[1][1],
              //              timestamp_list[1][2], timestamp_list[1][3],
              //              timestamp_list[1][4], timestamp_list[1][5],
              //              timestamp_list[1][6], timestamp_list[1][7]);

              // UART1_Printf("%d %d %d %d %d %d %d %d %d\r\n", 2,
              //              timestamp_list[2][0], timestamp_list[2][1],
              //              timestamp_list[2][2], timestamp_list[2][3],
              //              timestamp_list[2][4], timestamp_list[2][5],
              //              timestamp_list[2][6], timestamp_list[2][7]);

              // UART1_Printf("%d %d %d %d %d %d %d %d %d\r\n", 3,
              //              timestamp_list[3][0], timestamp_list[3][1],
              //              timestamp_list[3][2], timestamp_list[3][3],
              //              timestamp_list[3][4], timestamp_list[3][5],
              //              timestamp_list[3][6], timestamp_list[3][7]);

              // UART1_Printf("%d %d %d %d %d %d %d %d %d\r\n", 4,
              //              timestamp_list[4][0], timestamp_list[4][1],
              //              timestamp_list[4][2], timestamp_list[4][3],
              //              timestamp_list[4][4], timestamp_list[4][5],
              //              timestamp_list[4][6], timestamp_list[4][7]);

              // if (h->traceflag)
              // {
              //   UART1_Printf("[sg_resp] %d %d %d %d %d %d %d\r\n", loop,
              //                (uint8_t)h->is_calibration,
              //                (uint8_t)trigger_index, (uint32_t)h->is_trigger,
              //                h->fusion_filter_value,
              //                (uint32_t)h->trigger_tick, (uint32_t)last_tick);
              //   UART1_Printf("%d %d %d %d %d\r\n", h->calibration_values[0],
              //                h->calibration_values[1],
              //                h->calibration_values[2],
              //                h->calibration_values[3],
              //                h->fusion_filter_value);
              // }
            }
            h->times_read--;
            last_sendf_tick = timer_read_time();
          } else {
            if (loop % 100 == 0) {
              UART1_Printf("repeated data, index:%d\r\n", (uint8_t)h->trigger_index);
            }
          }
          last_is_trigger = h->now_trigger;
          last_is_calibration = h->is_calibration;
          last_trigger_tick = h->trigger_tick;
        }
      }
    }
  }
}
DECL_TASK(hx711s_task);

/**
 * @description:
 * @author: your name
 * @return {*}
 */
void hx711s_shutdown(void) {
  uint8_t oid;
  struct Hx711s *h;
  foreach_oid(oid, h, command_config_hx711s) {
    h->times_read = 0;
    h->flags &= ~HX711S_START;
  }
  return;
}
DECL_SHUTDOWN(hx711s_shutdown);

/**
 * @description: Hx711 sample命令行调试接口
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void hx711s_cli(int argc, char *argv[]) {
  //参数配置

  if (0 == strcmp("-config", argv[0])) {
    if (0 == strcmp("-show", argv[1])) {
      UART1_Printf("\r\n");
      UART1_Printf("\r\n HX711S Config      :");
      UART1_Printf("\r\n oid                : %d", hx711s.oid);
      UART1_Printf("\r\n rest_ticks         : %d", hx711s.rest_ticks);
      UART1_Printf("\r\n flags              : %d", hx711s.flags);
      UART1_Printf("\r\n hx711_count        : %d", hx711s.hx711_count);
      UART1_Printf("\r\n times_read         : %d", hx711s.times_read);
      UART1_Printf("\r\n waketime           : %d",
                   hx711s.hx711s_timer.waketime);
      UART1_Printf("\r\n kalman_q           : %d %d %d %d", hx711s.kalman_q[0],
                   hx711s.kalman_q[1], hx711s.kalman_q[2], hx711s.kalman_q[3]);
      UART1_Printf("\r\n kalman_r           : %d %d %d %d", hx711s.kalman_r[0],
                   hx711s.kalman_r[1], hx711s.kalman_r[2], hx711s.kalman_r[3]);
      UART1_Printf("\r\n is_calibration     : %d", hx711s.is_calibration);
      UART1_Printf("\r\n sensor init value  : %d %d %d %d",
                   hx711s.init_values[0], hx711s.init_values[1],
                   hx711s.init_values[2], hx711s.init_values[3]);

    } else if (0 == strcmp("-set", argv[1])) {
      if (0 == strcmp("-rest_ticks", argv[2])) {
        hx711s.rest_ticks = atoi(argv[3]);
        UART1_Printf("\r\n rest_ticks     : %d", hx711s.rest_ticks);
      } else if (0 == strcmp("-hx711_count", argv[2])) {
        hx711s.hx711_count = atoi(argv[3]);
        UART1_Printf("\r\n hx711_count     : %d", hx711s.hx711_count);
      } else if (0 == strcmp("-times_read", argv[2])) {
        hx711s.times_read = atoi(argv[3]);
        UART1_Printf("\r\n times_read     : %d", hx711s.times_read);
      } else if (0 == strcmp("-waketime", argv[2])) {
        hx711s.hx711s_timer.waketime = atoi(argv[3]);
        UART1_Printf("\r\n waketime     : %d", hx711s.hx711s_timer.waketime);
      } else if (0 == strcmp("-kalman_q", argv[2])) {
        hx711s.kalman_q[0] = atoi(argv[3]);
        hx711s.kalman_q[1] = atoi(argv[4]);
        hx711s.kalman_q[2] = atoi(argv[5]);
        hx711s.kalman_q[3] = atoi(argv[6]);
        UART1_Printf("\r\n kalman_q:  %d %d %d %d ", hx711s.kalman_q[0],
                     hx711s.kalman_q[1], hx711s.kalman_q[2],
                     hx711s.kalman_q[3]);

        // 参数重置
        // hx711_ekf_init(&hx711_sample_ekf_t, &hx711s);
      } else if (0 == strcmp("-kalman_r", argv[2])) {
        hx711s.kalman_r[0] = atoi(argv[3]);
        hx711s.kalman_r[1] = atoi(argv[4]);
        hx711s.kalman_r[2] = atoi(argv[5]);
        hx711s.kalman_r[3] = atoi(argv[6]);

        // 参数重置
        // hx711_ekf_init(&hx711_sample_ekf_t, &hx711s);
        UART1_Printf("\r\n kalman_r:  %d %d %d %d ", hx711s.kalman_r[0],
                     hx711s.kalman_r[1], hx711s.kalman_r[2],
                     hx711s.kalman_r[3]);
      } else {
        UART1_Printf("\r\nNOTICE: hx711_cli para:%s err.", argv[2]);
      }
    } else {
      UART1_Printf("\r\nNOTICE: hx711_cli -config:%s err.", argv[1]);
    }
  }
  // 数据
  else if (0 == strcmp("-data", argv[0])) {
    UART1_Printf("\r\nsensor1 init value  : %d", hx711s.init_values[0]);
    UART1_Printf("\r\nsensor2 init value  : %d", hx711s.init_values[1]);
    UART1_Printf("\r\nsensor3 init value  : %d", hx711s.init_values[2]);
    UART1_Printf("\r\nsensor4 init value  : %d", hx711s.init_values[3]);
    UART1_Printf("\r\nsensor1 value       : %d", hx711s.sample_values[0]);
    UART1_Printf("\r\nsensor2 value       : %d", hx711s.sample_values[1]);
    UART1_Printf("\r\nsensor3 value       : %d", hx711s.sample_values[2]);
    UART1_Printf("\r\nsensor4 value       : %d", hx711s.sample_values[3]);
    UART1_Printf("\r\nsensor1 filter value: %d", hx711s.filter_values[0]);
    UART1_Printf("\r\nsensor2 filter value: %d", hx711s.filter_values[1]);
    UART1_Printf("\r\nsensor3 filter value: %d", hx711s.filter_values[2]);
    UART1_Printf("\r\nsensor4 filter value: %d", hx711s.filter_values[3]);
    UART1_Printf("\r\nfusion filter value : %d", hx711s.fusion_filter_value);
  }
  // 开启采集
  else if (0 == strcmp("-start", argv[0])) {
    // uint32_t args[5] = {0, 0, 0, 0, 1};
    // command_sg_probe_check(args);
  }
  // 停止采集
  else if (0 == strcmp("-stop", argv[0])) {
    // uint32_t args[5] = {0, 0, 0, 0, 2};
    // command_sg_probe_check(args);

  }
  // 标定： hx711s -calibration 50
  else if (0 == strcmp("-calibration", argv[0])) {
    // uint32_t args[5] = {0, argv[1], 0, 0, 2};
    // command_calibration_sample(args);
    hx711s.times_read = atoi(argv[1]);
    if (hx711s.times_read < 1) {
      hx711s.times_read = 1;
    }
    uint8_t choosed_sensor = 0;
    struct TimeStructure ptime;
    unsigned int timestamp_ms = 0;
    unsigned int last_timestamp_ms = 0;
    uint32_t loop = 0;
    hx711s.traceflag |= 0x04;

    while (1) {
      rtc_gettime(&ptime);
      timestamp_ms = rtc_secfromdate(&ptime, DATE_BEIJING);

      if (timestamp_ms - last_timestamp_ms > 5) {
        last_timestamp_ms = timestamp_ms;
        choosed_sensor++;
        // 1.选择sensor
        if (choosed_sensor >= hx711s.hx711_count) {
          choosed_sensor = 0;
        }
        calibration_strain_gauge(&hx711s, choosed_sensor);
        if (hx711s.is_calibration > 0) {
          if (hx711s.is_calibration == 0x80) {
            UART1_Printf("\r\nhx711s calibration ok!!");
          }
          if (hx711s.is_calibration & 0x01) {
            UART1_Printf("\r\nhx711s sensor 1 err!!!!!!");
          }
          if (hx711s.is_calibration & 0x02) {
            UART1_Printf("\r\nhx711s sensor 2 err!!!!!!");
          }
          if (hx711s.is_calibration & 0x04) {
            UART1_Printf("\r\nhx711s sensor 3 err!!!!!!");
          }
          if (hx711s.is_calibration & 0x08) {
            UART1_Printf("\r\nhx711s sensor 4 err!!!!!!");
          }
          break;
        }
        if (hx711s.times_read <= 0) {
          break;
        }
      }
    }
  }
  // 指针地址
  else if (0 == strcmp("-address", argv[0])) {
    UART1_Printf("\r\np_hx711s address: 0x%x", hx711s);
  }
  // 调试日志
  else if (0 == strcmp("-trace", argv[0])) {
    if (0 == strcmp("-sample_task", argv[1])) {
      hx711s.traceflag = atoi(argv[2]);
      UART1_Printf("\r\nhx711s_data.traceflag:%d", hx711s.traceflag);
    } else {
      hx711s.traceflag = !hx711s.traceflag;
      UART1_Printf("\r\nhx711s_cli.traceflag:%d", hx711s.traceflag);
    }
  } else {
    UART1_Printf("\r\nNOTICE: hx711s_cli para:%s err.", argv[0]);
  }
}