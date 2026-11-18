#include "bme680_internal.h"
#include "bme680_compensation.h"

float bme680_compensate_temperature(bme680_t *dev, uint32_t adc_temp) {
  float var1, var2;

  var1 = (((float)adc_temp / 16384.0f) -
          ((float)dev->calib.temp_calib.par_t1 / 1024.0f)) *
         ((float)dev->calib.temp_calib.par_t2);
  var2 = ((((float)adc_temp / 131072.0f) -
           ((float)dev->calib.temp_calib.par_t1 / 8192.0f)) *
          (((float)adc_temp / 131072.0f) -
           ((float)dev->calib.temp_calib.par_t1 / 8192.0f))) *
         ((float)dev->calib.temp_calib.par_t3 * 16.0f);
  dev->t_fine = (var1 + var2);
  return (float)(dev->t_fine / 5120.0f);
}

float bme680_compensate_pressure(bme680_t *dev, uint32_t adc_press) {
  float var1, var2, var3;
  float pressure_comp;

  var1 = ((float)dev->t_fine / 2.0f) - 64000.0f;
  var2 = var1 * var1 * ((float)dev->calib.press_calib.par_p6) / 131072.0f;
  var2 = var2 + (var1 * ((float)dev->calib.press_calib.par_p5) * 2.0f);
  var2 = (var2 / 4.0f) + (((float)dev->calib.press_calib.par_p4) * 65536.0f);
  var3 = (((float)dev->calib.press_calib.par_p3) * var1 * var1) / 16384.0f;
  var1 = (var3 + (((float)dev->calib.press_calib.par_p2) * var1)) / 524288.0f;
  var1 = (1.0f + (var1 / 32768.0f)) * ((float)dev->calib.press_calib.par_p1);
  if (var1 == 0.0f) {
    return 0.0f;
  }
  pressure_comp = (1048576.0f - (float)adc_press);
  pressure_comp = ((pressure_comp - (var2 / 4096.0f)) * 6250.0f) / var1;
  var1 =
      (((float)dev->calib.press_calib.par_p9) * pressure_comp * pressure_comp) /
      2147483648.0f;
  var2 = pressure_comp * (((float)dev->calib.press_calib.par_p8) / 32768.0f);
  var3 = (pressure_comp / 256.0f) * (pressure_comp / 256.0f) *
         (pressure_comp / 256.0f) *
         ((float)dev->calib.press_calib.par_p10 / 131072.0f);
  pressure_comp =
      pressure_comp +
      ((var1 + var2 + var3 +
        ((float)dev->calib.press_calib.par_p7 * 128.0f)) /
       16.0f);
  return pressure_comp;
}

float bme680_compensate_humidity(bme680_t *dev, uint32_t adc_hum) {
  float temp_comp = dev->t_fine / 5120.0f;
  float var1 = (float)adc_hum -
               (((float)dev->calib.hum_calib.par_h1 * 16.0f) +
                (((float)dev->calib.hum_calib.par_h3 / 2.0f) * temp_comp));
  float var2 =
      var1 *
      (((float)dev->calib.hum_calib.par_h2 / 262144.0f) *
       (1.0f + (((float)dev->calib.hum_calib.par_h4 / 16384.0f) * temp_comp) +
        (((float)dev->calib.hum_calib.par_h5 / 1048576.0f) * temp_comp *
         temp_comp)));
  float var3 = (float)dev->calib.hum_calib.par_h6 / 16384.0f;
  float var4 = (float)dev->calib.hum_calib.par_h7 / 2097152.0f;
  float humidity_comp = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
  if (humidity_comp > 100.0f)
    humidity_comp = 100.0f;
  else if (humidity_comp < 0.0f)
    humidity_comp = 0.0f;
  return humidity_comp;
}
