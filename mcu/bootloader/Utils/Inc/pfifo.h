
#ifndef _PFIFO_H_
#define _PFIFO_H_
#include "stdint.h"
typedef struct
{
    uint8_t *buffer;         
    uint32_t in;          
    uint32_t out;        
    uint16_t size;  
    uint16_t error;
    uint16_t last_cnt;  
}fifo_rx_def_t;
int pfifo_init(fifo_rx_def_t *pfifo, uint8_t *buff, uint32_t size);
uint32_t pfifo_read_buff(fifo_rx_def_t *pfifo, uint8_t* buffer, uint32_t len);
uint32_t pfifo_write_buff(fifo_rx_def_t *pfifo, uint8_t *buffer, uint32_t len);
#endif 
