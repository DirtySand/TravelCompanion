#ifndef BME680_COMPENSATION_H
#define BME680_COMPENSATION_H

#include "bme680.h"
float bme680_compensate_temperature(bme680_t *dev, uint32_t adc_temp);
float bme680_compensate_pressure(bme680_t *dev, uint32_t adc_press);
float bme680_compensate_humidity(bme680_t *dev, uint32_t adc_hum);

#endif // BME680_COMPENSATION_H