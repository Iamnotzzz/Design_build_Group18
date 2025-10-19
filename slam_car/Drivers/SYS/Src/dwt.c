#include "dwt.h"

/**
  * @brief  初始化DWT的Cycle Counter
  * @param  无
  * @retval 0: 初始化失败; 1: 初始化成功
  */
uint32_t DWT_count = 0;
uint32_t last_DWT_CYCCNT = 0;

uint8_t DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    DWT_count = 0;
    last_DWT_CYCCNT = 0;

    // 验证 CYCCNT 是否递增
    if(DWT->CYCCNT) return 1; // 初始化成功
    else return 0; // 失败
}

uint32_t get_DWT_CYCCNT(void)
{
    return DWT->CYCCNT;
}

double DWT_Get_s(void)
{
    uint32_t cur = DWT->CYCCNT;
    uint32_t count = DWT_count;

    // 如果当前 CYCCNT < last_DWT_CYCCNT，说明刚刚溢出但还没进入中断
    if(cur < last_DWT_CYCCNT)
    {
        count++;
    }

    // 计算总周期数（64位）
    uint64_t total_cycles = ((uint64_t)count << 32) + cur;

    // 转换为秒
    return (double)total_cycles / ((double)SystemCoreClock);
}

double DWT_Get_ms(void)
{
    uint32_t cur = DWT->CYCCNT;
    uint32_t count = DWT_count;

    // 如果当前 CYCCNT < last_DWT_CYCCNT，说明刚刚溢出但还没进入中断
    if(cur < last_DWT_CYCCNT)
    {
        count++;
    }

    // 计算总周期数（64位）
    uint64_t total_cycles = ((uint64_t)count << 32) + cur;

    // 转换为毫秒
    return (double)total_cycles / ((double)SystemCoreClock / 1000.0f);
}

double DWT_Get_us(void)
{
    uint32_t cur = DWT->CYCCNT;
    uint32_t count = DWT_count;

    // 如果当前 CYCCNT < last_DWT_CYCCNT，说明刚刚溢出但还没进入中断
    if(cur < last_DWT_CYCCNT)
    {
        count++;
    }

    // 计算总周期数（64位）
    uint64_t total_cycles = ((uint64_t)count << 32) + cur;

    // 转换为微秒
    return (double)total_cycles / ((double)SystemCoreClock / 1000000.0f);
}

void DWT_Get2pointer_ms(unsigned long* tim_count)
{
    uint32_t cur = DWT->CYCCNT;
    uint32_t count = DWT_count;

    // 如果当前 CYCCNT < last_DWT_CYCCNT，说明刚刚溢出但还没进入中断
    if(cur < last_DWT_CYCCNT)
    {
        count++;
    }

    // 计算总周期数（64位）
    uint64_t total_cycles = ((uint64_t)count << 32) + cur;

    // 转换为毫秒
    *tim_count = total_cycles / (SystemCoreClock / 1000);
}

uint32_t DWT_Get_32us(void)
{
    uint32_t cur = DWT->CYCCNT;
    uint32_t count = DWT_count;

    // 如果当前 CYCCNT < last_DWT_CYCCNT，说明刚刚溢出但还没进入中断
    if(cur < last_DWT_CYCCNT)
    {
        count++;
    }

    // 计算总周期数（64位）
    uint64_t total_cycles = ((uint64_t)count << 32) + cur;

    // 转换为毫秒
    return (uint32_t)(total_cycles / (SystemCoreClock / 1000));
}
