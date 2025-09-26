#include "filter.h"

struct HighPassFilterParam high_pass_filter_param;
struct HighPassFilterParam fusion_high_pass_filter_param;

/**
 * @description:  初始化一个高通滤波器: 设定目标频率80Hz，截止频率为2Hz
 * @author: your name
 * @param {float} cut_frq_hz 滤波器的截止频率
 * @param {float} acq_frq_hz 滤波器的采样频率
 * @param {int} base_val  滤波器的初始值
 */
void high_pass_filter_init(float cut_frq_hz, float acq_frq_hz, int base_val) {
  high_pass_filter_param.vi = base_val;
  high_pass_filter_param.vi_prev = base_val;
  high_pass_filter_param.vo_prev = 0;
  high_pass_filter_param.vo = 0;
  // high pass filter @cutoff frequency = XX Hz
  high_pass_filter_param.cutoff_frq_hz = cut_frq_hz;
  high_pass_filter_param.acq_frq_hz = acq_frq_hz;

  UART1_Printf(
      "[INFO]high_pass_filter_init, base_val: %d cut_frq_hz：%f acq_frq_hz:%f "
      "\r\n",
      base_val, cut_frq_hz, acq_frq_hz);
  // execute XX every second
}

/**
 * @description:  对一个新的采集数据进行滤波，并返回滤波结果
 * @author: your name
 * @param {int} new_val  滤波前的数据
 * @return {int}  滤波后的数据
 */
int high_pass_filter(int new_val) {
  float rc, coff;
  high_pass_filter_param.vi = new_val;
  rc = 1.0f / 2.0f / PI / high_pass_filter_param.cutoff_frq_hz;
  coff = rc / (rc + 1 / high_pass_filter_param.acq_frq_hz);
  high_pass_filter_param.vo =
      (float)(high_pass_filter_param.vi - high_pass_filter_param.vi_prev +
              high_pass_filter_param.vo_prev) *
      coff;
  high_pass_filter_param.vo_prev = high_pass_filter_param.vo;  // update
  high_pass_filter_param.vi_prev = high_pass_filter_param.vi;

  // UART1_Printf("%f %f %d %d \r\n", rc, coff, new_val,
  //              fusion_high_pass_filter_param.vo);

  return high_pass_filter_param.vo;
}

/**
 * @description:  初始化一个高通滤波器: 设定目标频率80Hz，截止频率为2Hz
 * @author: your name
 * @param {float} cut_frq_hz 滤波器的截止频率
 * @param {float} acq_frq_hz 滤波器的采样频率
 * @param {int} base_val  滤波器的初始值
 */
void fusion_high_pass_filter_init(float cut_frq_hz, float acq_frq_hz,
                                  int base_val) {
  fusion_high_pass_filter_param.vi = base_val;
  fusion_high_pass_filter_param.vi_prev = base_val;
  fusion_high_pass_filter_param.vo_prev = 0;
  fusion_high_pass_filter_param.vo = 0;
  // high pass filter @cutoff frequency = XX Hz
  fusion_high_pass_filter_param.cutoff_frq_hz = cut_frq_hz;
  fusion_high_pass_filter_param.acq_frq_hz = acq_frq_hz;
  // execute XX every second
  UART1_Printf(
      "[INFO]fusion_high_pass_filter_init, base_val: %d cut_frq_hz：%f "
      "acq_frq_hz:%f "
      "\r\n",
      base_val, cut_frq_hz, acq_frq_hz);
}

/**
 * @description:  对一个新的采集数据进行滤波，并返回滤波结果
 * @author: your name
 * @param {int} new_val  滤波前的数据
 * @return {int}  滤波后的数据
 */
int fusion_high_pass_filter(int new_val) {
  float rc, coff;
  fusion_high_pass_filter_param.vi = new_val;
  rc = 1.0f / 2.0f / PI / fusion_high_pass_filter_param.cutoff_frq_hz;
  coff = rc / (rc + 1 / fusion_high_pass_filter_param.acq_frq_hz);
  fusion_high_pass_filter_param.vo =
      (float)(fusion_high_pass_filter_param.vi -
              fusion_high_pass_filter_param.vi_prev +
              fusion_high_pass_filter_param.vo_prev) *
      coff;
  fusion_high_pass_filter_param.vo_prev =
      fusion_high_pass_filter_param.vo;  // update
  fusion_high_pass_filter_param.vi_prev = fusion_high_pass_filter_param.vi;

  // UART1_Printf("%f %f %d %d \r\n", rc, coff, new_val,
  //              fusion_high_pass_filter_param.vo);

  return fusion_high_pass_filter_param.vo;
}