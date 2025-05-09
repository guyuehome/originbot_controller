#include "inv_time.h"

void inv_imu_sleep_us(uint32_t us)
{
	delay_us(us);
}

extern u32 g_start_count;
uint64_t inv_imu_get_time_us(void)
{
    return g_start_count*10*1000; //转成微妙
}

