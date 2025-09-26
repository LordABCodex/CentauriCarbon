#ifndef __LOCAL_UTIL_H
#define __LOCAL_UTIL_H

#include <stdio.h>

#define MAX_SENSOR_NUM 5  //使用滤波时的传感器数量: 4个sensor + 1个fision
#define MAX_DATA_NUM 30      //最大采样点数量：取偶数
#define WINDOW_DATA_NUM 28   //窗口数据量
typedef struct PointStruct  //点的结构
{
  double x;
  double y;
} Point;

extern int data_list[MAX_SENSOR_NUM][MAX_DATA_NUM];
extern int filter_data_list[MAX_SENSOR_NUM][MAX_DATA_NUM];
extern float data_list_f[MAX_SENSOR_NUM][MAX_DATA_NUM];
extern unsigned int timestamp_list[MAX_SENSOR_NUM][MAX_DATA_NUM];

extern float util_fabs(float data);
extern int util_abs(int data);

extern void bubble_sort(int *array, int len);
extern void bubble_sort_f(float *array, int len);

extern int median_filter(int *array, int len);
extern float median_filter_f(float *array, int len);

extern void array_sliding_window(int *array, int data_num, int data,
                                 uint8_t is_fifo);
extern void array_sliding_window_u(unsigned int *array, int data_num,
                                   unsigned int data, uint8_t is_fifo);

extern void array_sliding_window_f(float *array, int data_num, float data,
                                   uint8_t is_fifo);

extern float sliding_window_avg_filter_f(int index, int max_data_num,
                                         int window_data_num, float data,
                                         unsigned int timestamp,
                                         uint8_t is_fifo);
extern int sliding_window_avg_filter(int index, int max_data_num,
                                     int window_data_num, int data,
                                     unsigned int timestamp, uint8_t is_fifo);

extern int sliding_window_avg_exception_filter(int index, int max_data_num,
                                               int window_data_num, int *data,
                                               unsigned int timestamp,
                                               int exception_th,
                                               uint8_t is_fifo,
                                               uint8_t enable_hpf);
extern int sliding_window_avg_exception_filter_f(
    int index, int max_data_num, int window_data_num, float *data,
    unsigned int timestamp, float exception_th, uint8_t is_fifo,
    uint8_t enable_hpf);

extern void least_square_method1(int x[], int y[], int num, float *k0,
                                 float *b0);
extern void least_square_method2(int y[], int num, float *k0, float *b0);

extern void hx711_least_square(int y[], int num, float *k0, float *b0, int th);

#endif