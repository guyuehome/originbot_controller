/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

? ? http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/
#include "bsp_icm42670P.h"


static int myi2c_write(struct inv_imu_serif * serif, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int myi2c_read(struct inv_imu_serif * serif, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static void event_cb(inv_imu_sensor_event_t *event);

static struct inv_imu_device icm_driver;

// This is used by the event callback (not object aware), declared static
static inv_imu_sensor_event_t* event;


static void event_cb(inv_imu_sensor_event_t *evt) {
  memcpy(event,evt,sizeof(inv_imu_sensor_event_t));
}


int8_t icm42670_init(void)
{
	struct inv_imu_serif icm_serif;
  int rc = 0;
  uint8_t who_am_i = 0x00;
	
	
	IIC_icm42607_init();//i2c初始化
	
	
	icm_serif.serif_type = UI_I2C;
	icm_serif.read_reg  = myi2c_read;
	icm_serif.write_reg = myi2c_write;
	
	icm_serif.context   = 0;        /* no need */
  icm_serif.max_read  = 1024*1; /* maximum number of bytes allowed per serial read */
  icm_serif.max_write = 1024*1; /* maximum number of bytes allowed per serial write */
	
  rc = inv_imu_init(&icm_driver, &icm_serif, NULL);
	
	if (rc != INV_ERROR_SUCCESS) {
    return rc;
  }
  icm_driver.sensor_event_cb = event_cb;
	
	/* Check WHOAMI */
  rc = inv_imu_get_who_am_i(&icm_driver, &who_am_i);
  if(rc != 0) {
    return -2;
  }
  if (who_am_i != INV_IMU_WHOAMI)  {
    return -3;
  }
	
	

	rc = run_self_test();//自检
	
//	rc |= inv_imu_configure_fifo(&icm_driver, INV_IMU_FIFO_DISABLED); //关闭此功能
	
	
	startAccel(100,4);//50 8g量程
	startGyro(100,2000);//50hz 2000量程
	
	Delay_Ms(100);//等待imu配置完成
	
	
	
  // successful init, return 0
  return rc;

}

//获取角速度、线速度数据函数
int getDataFromRegisters(inv_imu_sensor_event_t* evt) {
  if(evt != NULL) {
    event = evt;
    return inv_imu_get_data_from_registers(&icm_driver);
  } else {
    return -1;
  }
}





static int myi2c_write(struct inv_imu_serif * serif, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	for(uint8_t i = 0; i < wlen; i++)
	{
		IIC_WriteReg(reg+i,wbuffer[i]); //里面是每写一个数据结束一次总线
	}
	
	return 0;
}

static int myi2c_read(struct inv_imu_serif * serif, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	for(uint8_t i = 0; i < rlen; i++)
	{
		rbuffer[i] = IIC_ReadReg(reg+i); 
	}
	return 0;
}

//配置加速度计的量程

int startAccel(uint16_t odr, uint16_t fsr) {
  int rc = 0;
  rc |= inv_imu_set_accel_fsr(&icm_driver, accel_fsr_g_to_param(fsr));
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_freq_to_param(odr));
  rc |= inv_imu_enable_accel_low_noise_mode(&icm_driver);
  return rc;
}

//配置陀螺仪计的量程
int startGyro(uint16_t odr, uint16_t fsr) {
  int rc = 0;
  rc |= inv_imu_set_gyro_fsr(&icm_driver, gyro_fsr_dps_to_param(fsr));
  rc |= inv_imu_set_gyro_frequency(&icm_driver, gyro_freq_to_param(odr));
  rc |= inv_imu_enable_gyro_low_noise_mode(&icm_driver);
  return rc;
}


ACCEL_CONFIG0_ODR_t accel_freq_to_param(uint16_t accel_freq_hz) {
  ACCEL_CONFIG0_ODR_t ret = ACCEL_CONFIG0_ODR_100_HZ;

  switch(accel_freq_hz) {
  case 12:   ret = ACCEL_CONFIG0_ODR_12_5_HZ;  break;
  case 25:   ret = ACCEL_CONFIG0_ODR_25_HZ;  break;
  case 50:   ret = ACCEL_CONFIG0_ODR_50_HZ;  break;
  case 100:  ret = ACCEL_CONFIG0_ODR_100_HZ; break;
  case 200:  ret = ACCEL_CONFIG0_ODR_200_HZ; break;
  case 400:  ret = ACCEL_CONFIG0_ODR_400_HZ; break;
  case 800:  ret = ACCEL_CONFIG0_ODR_800_HZ; break;
  case 1600: ret = ACCEL_CONFIG0_ODR_1600_HZ;  break;
  default:
    /* Unknown accel frequency. Set to default 100Hz */
    break;
  }
  return ret;
}

GYRO_CONFIG0_ODR_t gyro_freq_to_param(uint16_t gyro_freq_hz) {
  GYRO_CONFIG0_ODR_t ret = GYRO_CONFIG0_ODR_100_HZ;

  switch(gyro_freq_hz) {
  case 12:   ret = GYRO_CONFIG0_ODR_12_5_HZ;  break;
  case 25:   ret = GYRO_CONFIG0_ODR_25_HZ;  break;
  case 50:   ret = GYRO_CONFIG0_ODR_50_HZ;  break;
  case 100:  ret = GYRO_CONFIG0_ODR_100_HZ; break;
  case 200:  ret = GYRO_CONFIG0_ODR_200_HZ; break;
  case 400:  ret = GYRO_CONFIG0_ODR_400_HZ; break;
  case 800:  ret = GYRO_CONFIG0_ODR_800_HZ; break;
  case 1600: ret = GYRO_CONFIG0_ODR_1600_HZ;  break;
  default:
    /* Unknown gyro ODR. Set to default 100Hz */
    break;
  }
  return ret;
}


ACCEL_CONFIG0_FS_SEL_t accel_fsr_g_to_param(uint16_t accel_fsr_g) {
  ACCEL_CONFIG0_FS_SEL_t ret = ACCEL_CONFIG0_FS_SEL_16g;

  switch(accel_fsr_g) {
  case 2:  ret = ACCEL_CONFIG0_FS_SEL_2g;  break;
  case 4:  ret = ACCEL_CONFIG0_FS_SEL_4g;  break;
  case 8:  ret = ACCEL_CONFIG0_FS_SEL_8g;  break;
  case 16: ret = ACCEL_CONFIG0_FS_SEL_16g; break;
  default:
    /* Unknown accel FSR. Set to default 16G */
    break;
  }
  return ret;
}

GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps) {
  GYRO_CONFIG0_FS_SEL_t ret = GYRO_CONFIG0_FS_SEL_2000dps;

  switch(gyro_fsr_dps) {
  case 250:  ret = GYRO_CONFIG0_FS_SEL_250dps;  break;
  case 500:  ret = GYRO_CONFIG0_FS_SEL_500dps;  break;
  case 1000: ret = GYRO_CONFIG0_FS_SEL_1000dps; break;
  case 2000: ret = GYRO_CONFIG0_FS_SEL_2000dps; break;
  default:
    /* Unknown gyro FSR. Set to default 2000dps" */
    break;
  }
  return ret;
}

int run_self_test(void)
{
	inv_imu_selftest_output_t     out;
	inv_imu_selftest_parameters_t params;
	int                           rc   = INV_ERROR_SUCCESS;
	static int                    iter = 0;
	rc |= inv_imu_init_selftest_parameters_struct(&icm_driver, &params);
	/* Update `params` if needed here */
	rc |= inv_imu_run_selftest(&icm_driver, params, &out);
	
	if (rc != INV_ERROR_SUCCESS) 
    {
			printf("[%u] An error occurred while running selftest (rc=%d)", iter, rc);
		} 
    else 
    {
		/* Print self-test status */
		//printf("[%u] Accel self-test %s", iter, out.accel_status == 1 ? "OK" : "KO");
		if (out.accel_status != 1) {
			printf("  - Accel X: %s", out.ax_status == 1 ? "OK" : "KO");
			printf("  - Accel Y: %s", out.ay_status == 1 ? "OK" : "KO");
			printf("  - Accel Z: %s", out.az_status == 1 ? "OK" : "KO");
			rc |= INV_ERROR;
		}
		//printf("[%u] Gyro self-test %s", iter, out.gyro_status == 1 ? "OK" : "KO");
		if (out.gyro_status != 1) {
			printf("  - Gyro X: %s", out.gx_status == 1 ? "OK" : "KO");
			printf("  - Gyro Y: %s", out.gy_status == 1 ? "OK" : "KO");
			printf("  - Gyro Z: %s", out.gz_status == 1 ? "OK" : "KO");
			rc |= INV_ERROR;
		}
		/* Check incomplete state */
		if (out.gyro_status & 0x2) {
			printf("[%u] Gyro self-test are incomplete.", iter);
			rc |= INV_ERROR;
		}
	}
	iter++;
	return rc;
}






