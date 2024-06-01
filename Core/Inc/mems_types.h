
#ifndef MEMS_TYPES_H_
#define MEMS_TYPES_H_

#include <stdint.h>


/** struct to store raw data from 3 MEMS */
typedef struct {
  int16_t raw_acc[3];
  int16_t raw_gyr[3];
  int16_t raw_mag[3];
} raw_data_t;

/* struct to store processed MEMS data with corresponding axis */
typedef struct mems_data_t {
  float x;
  float y;
  float z;
} mems_data_t;

/* struct to store unprocessed MEMS data with corresponding axis */
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} mems_raw_data_t;

/* struct to store MEMS data */
typedef struct {
  int buffer_size;
  int buffer_head;
  mems_raw_data_t buffer_data[8];
  mems_data_t filtered_data;
} mems_t;

/* struct to store MEMS naming info and factors needed to measurement unit conversion */
typedef struct {
  char name[4];
  char data_labels[3];
  float convert_factors[2];
} mems_display_info_t;

#endif /* MEMS_TYPES_H_ */
