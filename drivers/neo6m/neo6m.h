#ifndef NEO6M_H
#define NEO6M_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct neo6m neo6m_t;

typedef enum {
  NEO6M_OK,
  NEO6M_UART_ERROR,
  NEO6M_NULL_PTR,
} neo6m_status_t;

typedef enum {
  NEO6M_UPDATED_NONE = 0,
  NEO6M_UPDATED_RMC = 1u << 0,
  NEO6M_UPDATED_GGA = 1u << 1,
} neo6m_update_t;

typedef struct {
  bool fix_valid;
  bool time_valid;
  bool date_valid;
  bool position_valid;
  bool speed_valid;
  bool course_valid;
  uint8_t hours, minutes, seconds;
  uint8_t day, month, year;
  int32_t latitude_e7;
  int32_t longitude_e7;
  uint32_t speed_kmh_x10;
  uint16_t course_x100;
  bool hdop_valid;
  bool altitude_valid;
  uint8_t fix_quality;
  uint8_t satellites;
  uint16_t hdop_x100;
  int32_t altitude_dm;
} neo6m_data_t;

typedef struct {
  uint32_t sentences;
  uint32_t checksum_errors;
  uint32_t overflows;
} neo6m_stats_t;

typedef int (*neo6m_uart_read_fn)(void *ctx, uint8_t *data, size_t max_length);

neo6m_t *neo6m_create(void *ctx, neo6m_uart_read_fn uart_read);
neo6m_status_t neo6m_process(neo6m_t *dev, uint32_t *updated);
neo6m_status_t neo6m_get_data(const neo6m_t *dev, neo6m_data_t *data);
neo6m_status_t neo6m_get_stats(const neo6m_t *dev, neo6m_stats_t *stats);

#endif // NEO6M_H