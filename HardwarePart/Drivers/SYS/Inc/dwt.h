#ifndef __DWT_H
#define __DWT_H


#include "main.h"

uint8_t DWT_Init(void);
uint32_t get_DWT_CYCCNT(void);
double DWT_Get_s(void);
double DWT_Get_ms(void);
double DWT_Get_us(void);
void DWT_Get2pointer_ms(unsigned long* tim_count);
uint32_t DWT_Get_32us(void);

extern uint32_t DWT_count;
extern uint32_t last_DWT_CYCCNT;






#endif
