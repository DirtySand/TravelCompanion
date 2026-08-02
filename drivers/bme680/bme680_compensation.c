#include "bme680_internal.h"
#include "bme680_compensation.h"

float bme680_compensate_temperature(bme680_t *dev, uint32_t adc_temp) {
  float var1, var2;
  float temp_comp;

  var1 = (((float)adc_temp / 16384.0) -
          ((float)dev->calib.temp_calib.par_t1 / 1024.0)) *
         ((float)dev->calib.temp_calib.par_t2);
  var2 = ((((float)adc_temp / 131072.0) -
           ((float)dev->calib.temp_calib.par_t1 / 8192.0)) *
          (((float)adc_temp / 131072.0) -
           ((float)dev->calib.temp_calib.par_t1 / 8192.0))) *
         ((float)dev->calib.temp_calib.par_t3 * 16.0);
  dev->t_fine = (var1 + var2);
  return (float)(dev->t_fine / 5120.0);
}

float bme680_compensate_pressure(bme680_t *dev, uint32_t adc_press) {
  float var1, var2, var3;
  float pressure_comp;

  var1 = ((float)dev->t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((float)dev->calib.press_calib.par_p6) / 131072.0;
  var2 = var2 + (var1 * ((float)dev->calib.press_calib.par_p5) * 2.0);
  var2 = (var2 / 4.0) + (((float)dev->calib.press_calib.par_p4) * 65536.0);
  var3 = (((float)dev->calib.press_calib.par_p3) * var1 * var1) / 16384.0;
  var1 = (var3 + (((float)dev->calib.press_calib.par_p2) * var1)) / 524288.0;
  var1 = (1.0 + (var1 / 32768.0)) * ((float)dev->calib.press_calib.par_p1);
  if (var1 == 0.0) {
    return 0; // avoid division by zero
  }
  pressure_comp = (1048576.0 - (float)adc_press);
  pressure_comp = ((pressure_comp - (var2 / 4096.0)) * 6250.0) / var1;
  var1 =
      (((float)dev->calib.press_calib.par_p9) * pressure_comp * pressure_comp) /
      2147483648.0;
  var2 = pressure_comp * (((float)dev->calib.press_calib.par_p8) / 32768.0);
  pressure_comp =
      pressure_comp +
      ((var1 + var2 + ((float)dev->calib.press_calib.par_p7)) / 16.0);
  return pressure_comp;
}

float bme680_compensate_humidity(bme680_t *dev, uint32_t adc_hum) {
  float temp_comp = dev->t_fine / 5120.0;
  float var1 = (float)adc_hum -
               (((float)dev->calib.hum_calib.par_h1 * 16.0) +
                (((float)dev->calib.hum_calib.par_h3 / 2.0) * temp_comp));
  float var2 =
      var1 *
      (((float)dev->calib.hum_calib.par_h2 / 262144.0) *
       (1.0 + (((float)dev->calib.hum_calib.par_h4 / 16384.0) * temp_comp) +
        (((float)dev->calib.hum_calib.par_h5 / 1048576.0) * temp_comp *
         temp_comp)));
  float var3 = (float)dev->calib.hum_calib.par_h6 / 16384.0;
  float var4 = (float)dev->calib.hum_calib.par_h7 / 2097152.0;
  float humidity_comp = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
  if (humidity_comp > 100.0)
    humidity_comp = 100.0;
  else if (humidity_comp < 0.0)
    humidity_comp = 0.0;
  return humidity_comp;
}