#include "delay.h"
#include "dwt.h"

// Œ¢√Î—” ±
void Delay_us(uint32_t us)
{
    double start = DWT_Get_us();

    while( DWT_Get_us() - start < us ) __NOP();
}

// ∫¡√Î—” ±
void Delay_ms(uint32_t ms)
{
    Delay_us(1000 * ms);
}
