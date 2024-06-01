/*
 * imu_lib.c
 *
 *  Created on: Apr 28, 2024
 *
 *  Code made based on examples:
 *  https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm9ds1_STdC/examples/lsm9ds1_read_data_polling.c
 */

#include "imu_lib.h"

/** variables */

/** structs for storing handlers of MEMs registers and status of device */
stmdev_ctx_t dev_ctx_imu;
stmdev_ctx_t dev_ctx_mag;
lsm9ds1_status_t reg;

/** structs for storing data from MEMS in form of raw data, calculated axis data and resultant vector */
static raw_data_t raw_imu_data = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

/** struct for storing data from MEMS with corresponding names and labels */
static mems_t mems_data[] = {
  {8, 0, {{0.0f, 0.0f, 0.0f}}, {0.0f, 0.0f, 0.0f}}, //ACC
  {8, 0, {{0.0f, 0.0f, 0.0f}}, {0.0f, 0.0f, 0.0f}}, //GYR
  {8, 0, {{0.0f, 0.0f, 0.0f}}, {0.0f, 0.0f, 0.0f}}, //MAG
};

static mems_display_info_t mems_info[] = {
  { "ACC", {'x', 'y', 'z'}, {1.0f, 1/1000.0f}},      // convert_factors - [milli-G, G]
  { "GYR", {'x', 'y', 'z'},  {1.0f, (3.1459f/180)}}, // convert_factors - [dps, rad/s]
  { "MAG", {'x', 'y', 'z'}, {1.0f, 1/1000.0f}}       // convert_factors - [gauss, milligauss]
};

/** Gyroscope offset, calculated during gyro calibration */
static int16_t zero_rate_offset[3] = {0, 0, 0};
/** handlers for imu (ACC_GYR) and MAG devices */
static sensbus_t mag_bus =
{
  .hbus = &SENSOR_BUS,
	.i2c_address = LSM9DS1_MAG_I2C_ADD_H,
	.cs_port = 0,
	.cs_pin = MAG_ADD
};
static sensbus_t imu_bus =
{
	.hbus = &SENSOR_BUS,
	.i2c_address = LSM9DS1_IMU_I2C_ADD_H,
	.cs_port = 0,
	.cs_pin = ACC_ADD
};

/** text buffer for UART transmission */
static uint8_t tx_buffer[TX_BUFF_SIZE];

/** functions */

void add_buffer_data(mems_t *mems_object, int16_t x, int16_t y, int16_t z) {
  mems_raw_data_t *buffer_data = mems_object->buffer_data;

  buffer_data[mems_object->buffer_head].x = x;
  buffer_data[mems_object->buffer_head].y = y;
  buffer_data[mems_object->buffer_head].z = z;

  mems_object->buffer_head++;
  mems_object->buffer_head = mems_object->buffer_head % mems_object->buffer_size;
}


void iir_filter_sensor_data(mems_t *mems_object, float alpha) {
  mems_raw_data_t *buffer_data = mems_object->buffer_data;

  mems_object->filtered_data.x = alpha * buffer_data[1].x + (1.0f - alpha) * buffer_data[0].x;
  mems_object->filtered_data.y = alpha * buffer_data[1].y + (1.0f - alpha) * buffer_data[0].y;
  mems_object->filtered_data.z = alpha * buffer_data[1].z + (1.0f - alpha) * buffer_data[0].z;

  buffer_data[0].x = mems_object->filtered_data.x;
  buffer_data[0].y = mems_object->filtered_data.y;
  buffer_data[0].z = mems_object->filtered_data.z;
}


void avg_filter_sensor_data(mems_t *mems_object, float multiply_factor) {
  mems_raw_data_t *buffer_data = mems_object->buffer_data;
  int buffer_size = mems_object->buffer_size;

  mems_object->filtered_data.x = 0;
  mems_object->filtered_data.y = 0;
  mems_object->filtered_data.z =0;

  for (int i = 0; i < buffer_size; i++) {
    mems_object->filtered_data.x += (float_t) buffer_data[i].x * multiply_factor;
    mems_object->filtered_data.y += (float_t) buffer_data[i].y * multiply_factor;
    mems_object->filtered_data.z += (float_t) buffer_data[i].z * multiply_factor;
  }
  mems_object->filtered_data.x /= buffer_size;
  mems_object->filtered_data.y /= buffer_size;
  mems_object->filtered_data.z /= buffer_size;
}


void set_unfiltered_sensor_data(mems_t *mems_object, float multiply_factor) {
  mems_raw_data_t *buffer_data = mems_object->buffer_data;

  mems_object->filtered_data.x = buffer_data[0].x * multiply_factor;
  mems_object->filtered_data.y = buffer_data[0].y * multiply_factor;
  mems_object->filtered_data.z = buffer_data[0].z * multiply_factor;
}


int32_t platform_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}


int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  /** Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}


int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp,
                                 uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}


int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp,
                                 uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  /** Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}


void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
}


void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}


void lis9ds1_fast_init(void)
{
  lsm9ds1_id_t whoamI;
  uint8_t rst;

  /** Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write_imu;
  dev_ctx_imu.read_reg = platform_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /** Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write_mag;
  dev_ctx_mag.read_reg = platform_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /** Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
    while (1) {
      /** manage here device not found */
      printf("Error!");
    }
  }

  /** Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /** Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
      PROPERTY_ENABLE);

  #if defined(FULL_SCALE)
    /** Set full scale */
    lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
    lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
    lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

  #elif defined(MIN_SCALE)
    /** Set minimal scale */
    lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_2g);
    lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_245dps);
    lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_4Ga);
  #endif

  /** acc embedded filtering */
//  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
//  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_9);
//  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);

  /** gyr embedded filtering */
//  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_MEDIUM);
//  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
//  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_OUT);
//  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_OUT);

  /** Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_20Hz);

  sprintf((char *)tx_buffer, "Init passed\n" );
  tx_com(tx_buffer, strlen((char const *)tx_buffer));

}

void lis9ds1_read_raw_data(void)
{
  /** Read device status register */
  lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

  if ( reg.status_imu.xlda && reg.status_imu.gda )
  {
    /** Read imu data */
    memset(raw_imu_data.raw_acc, 0x00, 3 * sizeof(int16_t));
    memset(raw_imu_data.raw_gyr, 0x00, 3 * sizeof(int16_t));
    lsm9ds1_acceleration_raw_get(&dev_ctx_imu, raw_imu_data.raw_acc);
    lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, raw_imu_data.raw_gyr);
  }

  if ( reg.status_mag.zyxda )
  {
    /** Read magnetometer data */
    memset(raw_imu_data.raw_mag, 0x00, 3 * sizeof(int16_t));
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, raw_imu_data.raw_mag);
  }
}


void lis9ds1_read_data(void)
{
  lis9ds1_read_raw_data();

  /** Buffer array has raw data from sensors */
  /** After conversion raw data - ACC in mili g, GYR in degree per second, MAG in mili Gaus */

  add_buffer_data(
      &mems_data[0],
      raw_imu_data.raw_acc[0],
      raw_imu_data.raw_acc[1],
      raw_imu_data.raw_acc[2]
  );
  avg_filter_sensor_data(&mems_data[0], ACC_FACTOR);

  add_buffer_data(
      &mems_data[1],
      raw_imu_data.raw_gyr[0] + zero_rate_offset[0],
      raw_imu_data.raw_gyr[1] + zero_rate_offset[1],
      raw_imu_data.raw_gyr[2] + zero_rate_offset[2]
  );
  avg_filter_sensor_data(&mems_data[1], GYR_FACTOR);

  add_buffer_data(
      &mems_data[2],
      raw_imu_data.raw_mag[0],
      raw_imu_data.raw_mag[1],
      raw_imu_data.raw_mag[2]
  );
  avg_filter_sensor_data(&mems_data[2], MAG_FACTOR);
}

void calibrate_gyroscope(void) {
  int32_t gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
  int samples = 70;

  for (int i = 0; i < samples; i++) {
    lis9ds1_read_raw_data();
    gyroX_sum -= raw_imu_data.raw_gyr[0];
    gyroY_sum -= raw_imu_data.raw_gyr[1];
    gyroZ_sum -= raw_imu_data.raw_gyr[2];
    platform_delay(100);
  }

  zero_rate_offset[0] = (int16_t)(gyroX_sum / samples);
  zero_rate_offset[1] = (int16_t)(gyroY_sum / samples);
  zero_rate_offset[2] = (int16_t)(gyroZ_sum / samples);
}


mems_t* get_mems_t_struct(void)
{
  return mems_data;
}

mems_display_info_t* get_mems_info_struct(void)
{
  return mems_info;
}
