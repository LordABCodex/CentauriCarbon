#include "string.h"
#include "stdint.h"
#include "pfifo.h"
/**
 * 单生产者单消费者无锁环形缓冲队列
 */
// 定义一个宏，用于检查一个数是否是2的幂。为了把%操作转换成位运算&。
#define IS_POWER_OF_2(x) ((x) != 0 && (((x) & ((x)-1)) == 0))

// 定义一个函数，用于获取两个数中的较小值，不要用宏。
uint32_t min(uint32_t X, uint32_t Y)
{
  return ((X) > (Y) ? (Y) : (X));
}

// 初始化环形缓冲区
int pfifo_init(fifo_rx_def_t *pfifo, uint8_t *buff, uint32_t size)
{
  // 检查输入参数是否有效
  if (pfifo == NULL || buff == NULL)
  {
    return -1;
  }

  // 检查缓冲区大小是否是2的幂，这是一个必要条件，因为我们使用位运算来实现环形缓冲区
  if (!IS_POWER_OF_2(size))
  {
    return -1;
  }

  // 初始化环形缓冲区的各个字段
  pfifo->in = 0;
  pfifo->out = 0;
  pfifo->buffer = buff;
  pfifo->size = size;

  return 0;
}

// 从环形缓冲区中读取数据
uint32_t pfifo_read_buff(fifo_rx_def_t *pfifo, uint8_t *buffer, uint32_t len)
{
  uint32_t length;

  // 检查输入参数是否有效
  if (pfifo == NULL || pfifo->buffer == NULL || buffer == NULL)
  {
    return 0;
  }

  // 确保我们不会读取超过缓冲区中存在的数据
  len = min(len, pfifo->in - pfifo->out);

  // 首先，从缓冲区的当前输出位置开始，读取到缓冲区的末尾
  length = min(len, pfifo->size - (pfifo->out & (pfifo->size - 1)));
  memcpy(buffer, pfifo->buffer + (pfifo->out & (pfifo->size - 1)), length);

  // 然后，如果还有剩余的数据，从缓冲区的开始位置读取
  memcpy(buffer + length, pfifo->buffer, len - length);

  // 更新缓冲区的输出位置
  pfifo->out += len;
  return len;
}

// 向环形缓冲区中写入数据
uint32_t pfifo_write_buff(fifo_rx_def_t *pfifo, uint8_t *buffer, uint32_t len)
{
  uint32_t length;

  // 检查输入参数是否有效
  if (pfifo == NULL || pfifo->buffer == NULL || buffer == NULL)
  {
    return 0;
  }

  // 确保我们不会写入超过缓冲区剩余空间的数据
  len = min(len, pfifo->size - pfifo->in + pfifo->out);

  // 首先，从缓冲区的当前输入位置开始，写入到缓冲区的末尾
  length = min(len, pfifo->size - (pfifo->in & (pfifo->size - 1)));
  memcpy(pfifo->buffer + (pfifo->in & (pfifo->size - 1)), buffer, length);

  // 然后，如果还有剩余的数据，从缓冲区的开始位置写入
  memcpy(pfifo->buffer, buffer + length, len - length);

  // 更新缓冲区的输入位置
  pfifo->in += len;
  return len;
}
