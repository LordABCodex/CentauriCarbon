#ifndef __FILTER_H
#define __FILTER_H

#include <stdio.h>
#include <stdlib.h>

#include "local_util.h"

#define PI 3.14159

struct HighPassFilterParam {
  int vi;               //输入
  int vi_prev;          //上一次输入
  int vo;               //输出
  int vo_prev;          //上一次输输出
  float cutoff_frq_hz;  //截止频率
  float acq_frq_hz;     //滤波器采样频率
};

extern void high_pass_filter_init(float cut_frq_hz, float acq_frq_hz,
                                  int base_val);

extern int high_pass_filter(int new_val);

extern void fusion_high_pass_filter_init(float cut_frq_hz, float acq_frq_hz,
                                         int base_val);

extern int fusion_high_pass_filter(int new_val);

#endif