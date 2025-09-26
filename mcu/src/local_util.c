#include "local_util.h"

//定义各个传感器的数据采样点
int data_list[MAX_SENSOR_NUM][MAX_DATA_NUM] = {0};
int filter_data_list[MAX_SENSOR_NUM][MAX_DATA_NUM] = {0};
float data_list_f[MAX_SENSOR_NUM][MAX_DATA_NUM] = {0};

//定义各个传感器的数据采样点时间戳列表: FiFo
unsigned int timestamp_list[MAX_SENSOR_NUM][MAX_DATA_NUM] = {0};
float timestamp_list_f[MAX_SENSOR_NUM][MAX_DATA_NUM] = {0};

/**
 * @description:
 * @author: your name
 * @param {float} data
 * @return {*}
 */
float util_fabs(float data) {
  if (data < 0) {
    return -data;
  } else {
    return data;
  }
}

/**
 * @description:
 * @author: your name
 * @param {int} data
 * @return {*}
 */
int util_abs(int data) {
  if (data < 0) {
    return -data;
  } else {
    return data;
  }
}

/**
 * @description: 冒泡排序
 * @author: your name
 * @param {int} array
 * @param {int} len
 * @return {*}
 */
void bubble_sort(int *array, int len) {
  int temp;
  for (int i = 0; i < len - 1; i++) {
    for (int j = 0; j < len - 1 - i; j++) {
      if (array[j] > array[j + 1]) {
        temp = array[j];
        array[j] = array[j + 1];
        array[j + 1] = temp;
      }
    }
  }
}

/**
 * @description: 冒泡排序
 * @author: your name
 * @param {float} array
 * @param {int} len
 * @return {*}
 */
void bubble_sort_f(float *array, int len) {
  float temp;
  for (int i = 0; i < len - 1; i++) {
    for (int j = 0; j < len - 1 - i; j++) {
      if (array[j] > array[j + 1]) {
        temp = array[j];
        array[j] = array[j + 1];
        array[j + 1] = temp;
      }
    }
  }
}

/**
 * @description:
 * @author: your name
 * @param {int *} array
 * @param {int} len
 * @return {*}
 */
int median_filter(int *array, int len) {
  int i;
  int tmp;

  // 冒泡排序
  bubble_sort(array, len);
  // 计算中值
  if ((len & 1) > 0) {
    tmp = array[(len + 1) / 2];
  } else {
    tmp = (array[len / 2] + array[len / 2 + 1]) / 2;
  }

  return tmp;
}

/**
 * @description:
 * @author: your name
 * @param {float *} array
 * @param {int} len
 * @return {*}
 */
float median_filter_f(float *array, int len) {
  int i;
  float tmp;

  // 冒泡排序
  bubble_sort_f(array, len);
  // 计算中值
  if ((len & 1) > 0) {
    tmp = array[(len + 1) / 2];
  } else {
    tmp = (array[len / 2] + array[len / 2 + 1]) / 2;
  }

  return tmp;
}

/**
 * @description: 数组FiFo操作，保持数组中最新的 data_num 组数据
 * @author: your name
 * @param {float *} array: 要处理的数组
 * @param {int} data_num: 最大数据
 * @param {float} data: 新数据
 * @param {int} is_fifo: 是否FiFo存储
 * @return {bool} 结果
 */
void array_sliding_window_f(float *array, int data_num, float data,
                            uint8_t is_fifo) {
  if (data_num < 2 || data_num > MAX_DATA_NUM) {
    UART1_Printf("[WARNING]array_sliding_window param invalid\r\n");
    return;
  }
  if (is_fifo) {
    //数据采样点在采样窗口内移动，FIFO操作
    for (int i = data_num - 2; i >= 0; i--) {
      array[i + 1] = array[i];
    }
    array[0] = data;
  } else {
    for (int i = 0; i < data_num - 2; i++) {
      array[i] = array[i + 1];
    }
    array[data_num - 1] = data;
  }
}

/**
 * @description: 数组FiFo操作，保持数组中最新的 data_num 组数据
 * @author: your name
 * @param {int *} array: 要处理的数组
 * @param {int} data_num: 最大数据
 * @param {int} data: 新数据
 * @param {int} is_fifo: 是否FiFo存储
 * @return {bool} 结果
 */
void array_sliding_window(int *array, int data_num, int data, uint8_t is_fifo) {
  if (data_num < 2 || data_num > MAX_DATA_NUM) {
    UART1_Printf("[WARNING]array_sliding_window param invalid\r\n");
    return;
  }
  if (is_fifo) {
    //数据采样点在采样窗口内移动，FIFO操作
    for (int i = data_num - 2; i >= 0; i--) {
      array[i + 1] = array[i];
    }
    array[0] = data;
  } else {
    for (int i = 0; i < data_num - 1; i++) {
      array[i] = array[i + 1];
    }
    array[data_num - 1] = data;
    // UART1_Printf("c:%d %d %d %d %d %d %d %d\r\n", array[0], array[1],
    // array[2],
    //              array[3], array[4], array[5], array[6], array[7]);
  }
}

/**
 * @description: 数组FiFo操作，保持数组中最新的 data_num 组数据
 * @author: your name
 * @param {int *} array: 要处理的数组
 * @param {int} data_num: 最大数据
 * @param {int} data: 新数据
 * @param {int} is_fifo: 是否FiFo存储
 * @return {bool} 结果
 */
void array_sliding_window_u(unsigned int *array, int data_num,
                            unsigned int data, uint8_t is_fifo) {
  if (data_num < 2 || data_num > MAX_DATA_NUM) {
    UART1_Printf("[WARNING]array_sliding_window param invalid\r\n");
    return;
  }
  if (is_fifo) {
    //数据采样点在采样窗口内移动，FIFO操作
    for (int i = data_num - 2; i >= 0; i--) {
      array[i + 1] = array[i];
    }
    array[0] = data;
  } else {
    for (int i = 0; i < data_num - 1; i++) {
      array[i] = array[i + 1];
    }
    array[data_num - 1] = data;
    // UART1_Printf("c:%d %d %d %d %d %d %d %d\r\n", array[0], array[1],
    // array[2],
    //              array[3], array[4], array[5], array[6], array[7]);
  }
}

/**
 * @description: 滑窗均值滤波
 * @author: your name
 * @param {int} index: 数据通道
 * @param {int} max_data_num: 最大数据
 * @param {int} window_data_num: 窗口数据代销
 * @param {float} data: 数据
 * @return {float} 滤波值
 */
float sliding_window_avg_filter_f(int index, int max_data_num,
                                  int window_data_num, float data,
                                  unsigned int timestamp, uint8_t is_fifo) {
  static int data_num[MAX_SENSOR_NUM] = {0};  //定义记录传感器的采样点个数
  int i;
  float sum = 0;
  float out = 0;
  float array[MAX_DATA_NUM] = {0};
  if (max_data_num < 2 || max_data_num < window_data_num ||
      max_data_num > MAX_DATA_NUM || index >= MAX_SENSOR_NUM) {
    UART1_Printf("[WARNING]sliding_window_avg_filter param invalid\r\n");
    return out;
  }

  //数据采样点在采样窗口内移动，FIFO操作
  // for (i = max_data_num - 2; i >= 0; i--)
  //   data_list_f[index][i + 1] = data_list_f[index][i];

  // data_list_f[index][0] = data;
  array_sliding_window_f(data_list_f[index], max_data_num, data, is_fifo);
  array_sliding_window_u(timestamp_list[index], max_data_num, timestamp,
                         is_fifo);

  //数据采样点数量小于采样窗口长度，对采样窗口数据累加后进行平均值运算
  if (data_num[index] < max_data_num) {
    data_num[index]++;
    for (i = 0; i < data_num[index]; i++) {
      sum += data_list_f[index][i];
    }
    out = sum / data_num[index];
  }
  //数据采样点已填满采样窗口，进行排序后，去除n个最大值及最小值后，对滤波窗口内的数据累加后进行平均值运算
  else {
    for (i = 0; i < max_data_num; i++) {
      array[i] = data_list_f[index][i];
    }
    //调用冒泡排序函数
    bubble_sort(array, max_data_num);
    // start = REMOVE_MAXMIN_NUM
    //去除采样窗口内最大最小值的数量，这里去除两个最大和两个最小
    int start = (max_data_num - window_data_num) / 2;

    for (i = start; i < start + window_data_num; i++) {
      sum += array[i];
    }
    out = sum / window_data_num;
  }
  return out;
}

/**
 * @description: 滑窗均值滤波
 * @author: your name
 * @param {int} index: 数据通道
 * @param {int} max_data_num: 最大数据
 * @param {int} window_data_num: 窗口数据代销
 * @param {int} data: 数据
 * @return {float} 滤波值
 */
int sliding_window_avg_filter(int index, int max_data_num, int window_data_num,
                              int data, unsigned int timestamp,
                              uint8_t is_fifo) {
  static int data_num[MAX_SENSOR_NUM] = {0};  //定义记录传感器的采样点个数
  int i;
  int sum = 0;
  int out = 0;
  int array[MAX_DATA_NUM] = {0};
  if (max_data_num < 2 || max_data_num < window_data_num ||
      max_data_num > MAX_DATA_NUM || index >= MAX_SENSOR_NUM) {
    UART1_Printf("[WARNING]sliding_window_avg_filter param invalid\r\n");
    return out;
  }

  //数据采样点在采样窗口内移动，FIFO操作
  // for (i = max_data_num - 2; i >= 0; i--)
  //   data_list[index][i + 1] = data_list[index][i];

  // data_list[index][0] = data;
  array_sliding_window(data_list[index], max_data_num, data, is_fifo);
  array_sliding_window_u(timestamp_list[index], max_data_num, timestamp,
                         is_fifo);

  //数据采样点数量小于采样窗口长度，对采样窗口数据累加后进行平均值运算
  if (data_num[index] < max_data_num) {
    data_num[index]++;
    for (i = 0; i < data_num[index]; i++) {
      sum += data_list[index][i];
    }
    out = sum / data_num[index];
  }
  //数据采样点已填满采样窗口，进行排序后，去除n个最大值及最小值后，对滤波窗口内的数据累加后进行平均值运算
  else {
    for (i = 0; i < max_data_num; i++) {
      array[i] = data_list[index][i];
    }
    //调用冒泡排序函数
    bubble_sort(array, max_data_num);
    // start = REMOVE_MAXMIN_NUM
    //去除采样窗口内最大最小值的数量，这里去除两个最大和两个最小
    int start = (max_data_num - window_data_num) / 2;

    for (i = start; i < start + window_data_num; i++) {
      sum += array[i];
    }
    out = sum / window_data_num;
  }
  return out;
}

/**
 * @description: 滑窗均值滤波
 * @author: your name
 * @param {int} index: 数据通道
 * @param {int} max_data_num: 最大数据
 * @param {int} window_data_num: 窗口数据
 * @param {int*} data: 数据输入，结果输出
 * @param {unsigned int} timestamp: 时间戳
 * @param {float} exception_th: 异常值阈值
 * @param {int} is_fifo: 是否FiFo存储
 * @param {int} enable_hpf: 是否使能高通滤波
 * @return {int} 滤波值
 */
int sliding_window_avg_exception_filter(int index, int max_data_num,
                                        int window_data_num, int *data,
                                        unsigned int timestamp,
                                        int exception_th, uint8_t is_fifo,
                                        uint8_t enable_hpf) {
  static int data_num[MAX_SENSOR_NUM] = {0};  //定义记录传感器的采样点个数
  static int last_out[MAX_SENSOR_NUM] = {0};  //上一次滤波结果值
  static uint32_t loop = 0;
  static uint32_t err_cnt = 0;
  loop++;
  int i;
  int sum = 0;
  int out = 0;
  int array[MAX_DATA_NUM] = {0};
  if (max_data_num < 2 || max_data_num < window_data_num ||
      max_data_num > MAX_DATA_NUM || index >= MAX_SENSOR_NUM) {
    UART1_Printf("[WARNING]sliding_window_avg_filter param invalid\r\n");
    return 0;
  }
  int filter_data = *data;
  if (enable_hpf) {
    if (index == 4) {
      filter_data = fusion_high_pass_filter(*data);
    } else {
      filter_data = high_pass_filter(*data);
    }
    // UART1_Printf("hf %d %d %d\r\n", index, *data, tmp);
  }

  // UART1_Printf("%d %d %d %d %d %d %d %d %d %d %d %d %d  out:%d last_out:%d filter_data:%d %d\r\n", index, data_list[index][0],
  //               data_list[index][1], data_list[index][2],
  //               data_list[index][3], data_list[index][4],
  //               data_list[index][5], data_list[index][6],
  //               data_list[index][7],data_list[index][8],
  //               data_list[index][9],data_list[index][10],
  //               data_list[index][11],out,last_out[index],filter_data,exception_th);

  if (exception_th > 0) {
    int tmp = last_out[index] - filter_data;
    if ((util_abs(tmp) > exception_th) && (last_out[index] != 0)) {
      err_cnt++;
      // if (err_cnt%1000==0) 
      {
        UART1_Printf("[WARNING]sensor:%d %d, f_out:%d e:%d %d/%d\r\n", index, tmp,
                    last_out[index], filter_data, err_cnt, loop);
      }
      *data = last_out[index];
      return 0;
    }
  }


  //数据采样点在采样窗口内移动
  array_sliding_window(data_list[index], max_data_num, filter_data, is_fifo);
  array_sliding_window(filter_data_list[index], max_data_num, *data, is_fifo);
  array_sliding_window_u(timestamp_list[index], max_data_num, timestamp,
                         is_fifo);

  //数据采样点数量小于采样窗口长度，对采样窗口数据累加后进行平均值运算
  if (data_num[index] < max_data_num) {
    data_num[index]++;
    for (i = 0; i < data_num[index]; i++) {
      sum += data_list[index][i];
    }
    out = sum / data_num[index];
  }
  //数据采样点已填满采样窗口，进行排序后，去除n个最大值及最小值后，对滤波窗口内的数据累加后进行平均值运算
  else {
    for (i = 0; i < max_data_num; i++) {
      array[i] = data_list[index][i];
    }
    //调用冒泡排序函数
    bubble_sort(array, max_data_num);
    // start = REMOVE_MAXMIN_NUM
    //去除采样窗口内最大最小值的数量，这里去除两个最大和两个最小
    int start = (max_data_num - window_data_num) / 2;

    for (i = start; i < start + window_data_num; i++) {
      sum += array[i];
    }
    out = sum / window_data_num;
  }
  // if (loop % 11 == 0 || loop % 12 == 0 || loop % 13 == 0 || loop % 14 == 0 ||
  //     loop % 15 == 0) 
  // {
  //   UART1_Printf("data: %d %d %d %d %d %d %d %d %d %d %d %d %d out:%d last_out:%d\r\n", index,
  //                data_list[index][0], data_list[index][1],
  //                data_list[index][2], data_list[index][3],
  //                data_list[index][4], data_list[index][5],
  //                data_list[index][6], data_list[index][7],
  //                data_list[index][8], data_list[index][9],
  //                data_list[index][10], data_list[index][11],out,last_out[index]);
  // }
  if (out==0) {
    out=1;
  }
    // UART1_Printf("data: %d %d %d %d %d %d %d %d %d %d %d %d %d out:%d last_out:%d\r\n", index,
    //              data_list[index][0], data_list[index][1],
    //              data_list[index][2], data_list[index][3],
    //              data_list[index][4], data_list[index][5],
    //              data_list[index][6], data_list[index][7],
    //              data_list[index][8], data_list[index][9],
    //              data_list[index][10], data_list[index][11],out,last_out[index]);
  last_out[index] = out;
  return 1;
}

/**
 * @description: 滑窗均值滤波
 * @author: your name
 * @param {int} index: 数据通道
 * @param {int} max_data_num: 最大数据
 * @param {int} window_data_num: 窗口数据
 * @param {float*} data: 数据输入，结果输出
 * @param {float} exception_th: 异常值阈值
 * @param {int} is_fifo: 是否FiFo存储
 * @param {int} enable_hpf: 是否使能高通滤波
 * @return {int} 滤波值
 */
int sliding_window_avg_exception_filter_f(int index, int max_data_num,
                                          int window_data_num, float *data,
                                          unsigned int timestamp,
                                          float exception_th, uint8_t is_fifo,
                                          uint8_t enable_hpf) {
  static int data_num[MAX_SENSOR_NUM] = {0};  //定义记录传感器的采样点个数
  static float last_out[MAX_SENSOR_NUM] = {0};  //上一次滤波结果值
  int i;
  float sum = 0;
  float out = 0;
  float array[MAX_DATA_NUM] = {0};
  if (max_data_num < 2 || max_data_num < window_data_num ||
      max_data_num > MAX_DATA_NUM || index >= MAX_SENSOR_NUM) {
    UART1_Printf("[WARNING]sliding_window_avg_filter param invalid\r\n");
    return 0;
  }

  if (enable_hpf) {
    int tmp = 0;
    if (index == 4) {
      tmp = fusion_high_pass_filter(*data);
    } else {
      tmp = high_pass_filter(*data);
    }
    // UART1_Printf("hf %d %d %d\r\n", index, *data, tmp);
    *data = tmp;
  }

  if (exception_th > 0) {
    int tmp = last_out[index] - *data;
    if ((util_abs(tmp) > exception_th) && (last_out[index] != 0)) {
      // UART1_Printf("[WARNING]%d, f_out:%d   exception-value:%d \r\n", tmp,
      //              last_out[index], *data);
      // UART1_Printf("%d %d %d %d %d %d %d %d %d\r\n", index, data_list[index][0],
      //              data_list[index][1], data_list[index][2],
      //              data_list[index][3], data_list[index][4],
      //              data_list[index][5], data_list[index][6],
      //              data_list[index][7]);
      return 0;
    }
  }

  //数据采样点在采样窗口内移动，FIFO操作
  array_sliding_window_f(data_list_f[index], max_data_num, *data, is_fifo);
  array_sliding_window_u(timestamp_list[index], max_data_num, timestamp,
                         is_fifo);

  //数据采样点数量小于采样窗口长度，对采样窗口数据累加后进行平均值运算
  if (data_num[index] < max_data_num) {
    data_num[index]++;
    for (i = 0; i < data_num[index]; i++) {
      sum += data_list_f[index][i];
    }
    out = sum / data_num[index];
  }
  //数据采样点已填满采样窗口，进行排序后，去除n个最大值及最小值后，对滤波窗口内的数据累加后进行平均值运算
  else {
    for (i = 0; i < max_data_num; i++) {
      array[i] = data_list_f[index][i];
    }
    //调用冒泡排序函数
    bubble_sort(array, max_data_num);
    // start = REMOVE_MAXMIN_NUM
    //去除采样窗口内最大最小值的数量，这里去除两个最大和两个最小
    int start = (max_data_num - window_data_num) / 2;

    for (i = start; i < start + window_data_num; i++) {
      sum += array[i];
    }
    out = sum / window_data_num;
  }
  last_out[index] = out;
  return 1;
}

/**
 * @description: 最小二乘法线性拟合直线
 * @author: your name
 * @param {Point} p: 传入要线性拟合的点数据（结构体数组）
 * @param {int} num: 线性拟合的点的数量
 * @param {float} *k0: 直线斜率参数存放地址
 * @param {float} *b0: 直线截距参数存放地址
 * @return {*}
 */
void least_square_method(Point p[], int num, float *k0, float *b0) {
  int i = 0;
  float K = 0, b = 0, A = 0, B = 0, C = 0, D = 0;
  for (i = 0; i < num; i++) {
    A += p[i].x * p[i].y;
    B += p[i].x;
    C += p[i].y;
    D += p[i].x * p[i].x;
  }
  K = (num * A - B * C) / (num * D - B * B);
  b = C / num - K * B / num;
  *k0 = K;
  *b0 = b;
  if (b > 0) {
    UART1_Printf("[DEBUG]y=%fx+%f\n", K, b);
  } else if (b == 0) {
    UART1_Printf("[DEBUG]y=%fx\n", K);
  } else {
    UART1_Printf("[DEBUG]y=%fx%f\n", K, b);
  }
}

/**
 * @description: 最小二乘法线性拟合直线
 * @author: your name
 * @param {Point} p: 传入要线性拟合的点数据（结构体数组）
 * @param {int} num: 线性拟合的点的数量
 * @param {float} *k0: 直线斜率参数存放地址
 * @param {float} *b0: 直线截距参数存放地址
 * @return {*}
 */
void least_square_method1(int x[], int y[], int num, float *k0, float *b0) {
  int i = 0;
  float K = 0, b = 0, A = 0, B = 0, C = 0, D = 0;
  for (i = 0; i < num; i++) {
    A += x[i] * y[i];
    B += x[i];
    C += y[i];
    D += x[i] * x[i];
  }
  K = (num * A - B * C) / (num * D - B * B);
  b = C / num - K * B / num;
  *k0 = K;
  *b0 = b;
  if (b > 0) {
    UART1_Printf("[DEBUG]y=%fx+%f\n", K, b);
  } else if (b == 0) {
    UART1_Printf("[DEBUG]y=%fx\n", K);
  } else {
    UART1_Printf("[DEBUG]y=%fx%f\n", K, b);
  }
}

/**
 * @description: 最小二乘法线性拟合直线
 * @author: your name
 * @param {Point} p: 传入要线性拟合的点数据（结构体数组）
 * @param {int} num: 线性拟合的点的数量
 * @param {float} *k0: 直线斜率参数存放地址
 * @param {float} *b0: 直线截距参数存放地址
 * @return {*}
 */
void least_square_method2(int y[], int num, float *k0, float *b0) {
  int i = 0;
  float K = 0, b = 0, A = 0, B = 0, C = 0, D = 0;
  for (i = 0; i < num; i++) {
    A += i * y[i];
    B += i;
    C += y[i];
    D += i * i;
  }
  K = (num * A - B * C) / (num * D - B * B);
  b = C / num - K * B / num;
  *k0 = K;
  *b0 = b;
  // if (b > 0) {
  //   UART1_Printf("[DEBUG]y=%fx+%f\r\n", K, b);
  // } else if (b == 0) {
  //   UART1_Printf("[DEBUG]y=%fx\r\n", K);
  // } else {
  //   UART1_Printf("[DEBUG]y=%fx%f\r\n", K, b);
  // }
}

/**
 * @description: 最小二乘法线性拟合直线
 * @author: your name
 * @param {Point} p: 传入要线性拟合的点数据（结构体数组）
 * @param {int} num: 线性拟合的点的数量
 * @param {float} *k0: 直线斜率参数存放地址
 * @param {float} *b0: 直线截距参数存放地址
 * @return {*}
 */
void hx711_least_square(int y[], int num, float *k0, float *b0, int th) {
  int i = 0;
  float K = 0, b = 0, A = 0, B = 0, C = 0, D = 0;
  int half_num = num / 2;
  int first_hulf_sum = 0;
  int second_half_sum = 0;
  for (i = 0; i < num; i++) {
    if (i < half_num) {
      first_hulf_sum += util_abs(y[i]);
    } else {
      second_half_sum += util_abs(y[i]);
    }
    A += i * y[i];
    B += i;
    C += y[i];
    D += i * i;
  }
  // th 跳动阈值
  if ((first_hulf_sum + th) < second_half_sum) {
    if (second_half_sum > half_num * th) {
      // UART1_Printf("[sg resilience] %d %d %d %d %d %d %d %d\r\n", y[0], y[1],
      //              y[2], y[3], y[4], y[5], y[6], y[7]);
    }
  } else {
    K = (num * A - B * C) / (num * D - B * B);
    b = C / num - K * B / num;
    *k0 = K;
    *b0 = b;
  }
}
