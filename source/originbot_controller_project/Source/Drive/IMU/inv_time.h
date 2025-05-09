#ifndef __INV_TIME_H
#define __INV_TIME_H

#include <string.h>
#include <stdio.h>

#include "Main.h"
#include "delay.h"

void inv_imu_sleep_us(uint32_t us);

uint64_t inv_imu_get_time_us(void);



#endif



