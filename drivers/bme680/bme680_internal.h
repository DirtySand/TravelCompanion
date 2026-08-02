#ifndef bme680_internal_h
#define bme680_internal_h
#include "bme680.h"
#include <stdint.h>

struct bme680_calib_temperature {
  uint16_t par_t1;
  int16_t par_t2;
  int8_t par_t3;
};

struct bme680_calib_pressure {
  uint16_t par_p1;
  int16_t par_p2;
  int8_t par_p3;
  int16_t par_p4;
  int16_t par_p5;
  uint8_t par_p6;
  int8_t par_p7;
  int16_t par_p8;
  int16_t par_p9;
  uint8_t par_p10;
};

struct bme680_calib_humidity {
  uint16_t par_h1;
  uint16_t par_h2;
  int8_t par_h3;
  int8_t par_h4;
  int8_t par_h5;
  uint8_t par_h6;
  int8_t par_h7;
};

struct bme680_calib {
  struct bme680_calib_temperature temp_calib;
  struct bme680_calib_pressure press_calib;
  struct bme680_calib_humidity hum_calib;
};

struct bme680 {
  void *ctx;
  uint8_t dev_addr;
  struct bme680_calib calib;
  float t_fine;
  bme680_i2c_read_fn i2c_read;
  bme680_i2c_write_fn i2c_write;
};
#endif