/*
 * imu_lib.h
 *
 *  Created on: Apr 28, 2024
 *
 *  Code made based on examples:
 *  https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm9ds1_STdC/examples/lsm9ds1_read_data_polling.c
 */

#ifndef INC_IMU_LIB_H_
#define INC_IMU_LIB_H_

#include "mems_types.h"
#include <string.h>
#include <stdio.h>
#include "lsm9ds1_reg.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"



/** struct for identifying device ACC+GYR or MAG */
typedef struct {
  void   *hbus;
  uint8_t i2c_address;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} sensbus_t;

#define SENSOR_BUS hi2c1
static const uint8_t ACC_ADD = 0x6BU << 1;
static const uint8_t MAG_ADD = 0x1EU << 1;


#define    BOOT_TIME            20 /** time in miliseconds for proper run time of sensor */

/** change data resolution and range for minimum and maximum ranges */
//#define FULL_SCALE /* LSM9DS1_4g, LSM9DS1_2000dps, LSM9DS1_16Ga */
#define MIN_SCALE /* LSM9DS1_2g, LSM9DS1_245dps, LSM9DS1_4Ga */

#if defined(FULL_SCALE)
#define ACC_FACTOR 0.732f
#define GYR_FACTOR (70.0f / 1000.0f)
#define MAG_FACTOR 0.58f
#elif defined(MIN_SCALE)
#define ACC_FACTOR 0.061f
#define GYR_FACTOR (8.75f / 1000.0f)
#define MAG_FACTOR 0.14f
#endif

#define TX_BUFF_SIZE 1000


/** functions */


/**
 * @brief  Add x,y,z sensor measurement to the circular buffer array.
 *
 * @param  mems_object  Sensor object that stores the history of measurements in a buffer array
 * @param  x            x-axis measurement for the sensor
 * @param  y            y-axis measurement for the sensor
 * @param  z            z-axis measurement for the sensor
 *
 */
void add_buffer_data(mems_t *mems_object, int16_t x, int16_t y, int16_t z);
/**
 * @brief  Apply IIR filter to measurements from the buffer array.
 * 		     Save the filter output in the filtered_data attribute from mems_object.
 *
 * @param  mems_object  Sensor object that stores the history of measurements and filtered_data.
 * @param  alpha        Coefficient needed for iir filter
 *
 */
void iir_filter_sensor_data(mems_t *mems_object, float alpha);
/**
 * @brief  Apply moving average filter to measurements from the buffer array.
 * 		     Save the filter output in the filtered_data attribute from mems_object.
 *
 * @param  mems_object  Sensor object that stores the history of measurements and filtered_data.
 *
 */
void avg_filter_sensor_data(mems_t *mems_object, float multiply_factor);

/**
 * @brief  Save the last unfiltered sensor measurement to the filtered_data
 *
 * @param  mems_object  Sensor object that stores the history of measurements and filtered_data.
 */
void set_unfiltered_sensor_data(mems_t *mems_object, float multiply_factor);

/**
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_imu(void *handle, uint8_t reg,
    const uint8_t *bufp, uint16_t len);

/**
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_mag(void *handle, uint8_t reg,
    const uint8_t *bufp, uint16_t len);

/**
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_imu(void *handle, uint8_t reg,
    uint8_t *bufp, uint16_t len);

/**
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_mag(void *handle, uint8_t reg,
    uint8_t *bufp, uint16_t len);

/**
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
void tx_com( uint8_t *tx_buffer, uint16_t len );

/**
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms);

/**
 * @brief  Initialize device (accelerometer, gyroscope and magnetometer) and sets
 * 			settings for simplest continous data measurement at lowest ODR
 *
 */
void lis9ds1_fast_init(void);

/**
 * @brief  Read raw data from FIFO buffers into structs
 *
 */
void lis9ds1_read_raw_data(void);

/**
 * @brief  Read raw data from FIFO buffers into structs and convert values
 *      into standarized units (mg, dps, mG)
 *
 */
void lis9ds1_read_data(void);

/**
 * @brief  Calculate gyroscope offset to reduce gyroscope zero-rate drift.
 *
 */
void calibrate_gyroscope(void);

/**
 * @brief  Returns address of a struct which stores sensors measurements
 *
 */
mems_t* get_mems_t_struct(void);

/**
 * @brief  Returns address of a struct which stores information about used sensors
 *
 */
mems_display_info_t* get_mems_info_struct(void);


#endif /* INC_IMU_LIB_H_ */
