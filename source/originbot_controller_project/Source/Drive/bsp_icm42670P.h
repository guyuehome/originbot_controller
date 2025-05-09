#ifndef __BSP_ICM42670P_H
#define __BSP_ICM42670P_H

#include <string.h>
#include <stdio.h>

#include "Main.h"
#include "inv_imu_driver.h"
#include "inv_imu_transport.h"
#include "inv_imu_regmap_rev_a.h"
#include "inv_imu_selftest.h"

#define I2C_icm42670_ADDR     0xD0 //这个值已经是0x68<<1

int8_t icm42670_init(void);

ACCEL_CONFIG0_ODR_t accel_freq_to_param(uint16_t accel_freq_hz);
GYRO_CONFIG0_ODR_t gyro_freq_to_param(uint16_t gyro_freq_hz);

GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps);
ACCEL_CONFIG0_FS_SEL_t accel_fsr_g_to_param(uint16_t accel_fsr_g);

int startAccel(uint16_t odr, uint16_t fsr);
int startGyro(uint16_t odr, uint16_t fsr);
int getDataFromRegisters(inv_imu_sensor_event_t* evt) ;

int run_self_test(void);


#endif 

