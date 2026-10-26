#include "neo6m.h"
#include <stddef.h>
#include <string.h>

#define NEO6M_SENTENCE_MAX_CHARS 79
#define NEO6M_MAX_FIELDS 20
#define NEO6M_READ_CHUNK 32
#define NEO6M_CHECKSUM_DIGITS 2
#define NEO6M_ADDRESS_LENGTH 5

#define NEO6M_MINUTES_DECIMALS 5
#define NEO6M_MINUTES_SCALE 100000u
#define NEO6M_DEGREES_SCALE 10000000u
#define NEO6M_MAX_LATITUDE 90u
#define NEO6M_MAX_LONGITUDE 180u
#define NEO6M_METERS_PER_NAUTICAL_MILE 1852u
#define NEO6M_MAX_COURSE_X100 36000u

#define RMC_TIME 1
#define RMC_STATUS 2
#define RMC_LATITUDE 3
#define RMC_NS 4
#define RMC_LONGITUDE 5
#define RMC_EW 6
#define RMC_SPEED 7
#define RMC_COURSE 8
#define RMC_DATE 9

#define GGA_QUALITY 6
#define GGA_SATELLITES 7
#define GGA_HDOP 8
#define GGA_ALTITUDE 9

#define GSV_MESSAGES 1
#define GSV_MESSAGE_NUMBER 2
#define GSV_IN_VIEW 3
#define GSV_FIRST_CNO 7
#define GSV_BLOCK_FIELDS 4
#define NEO6M_MAX_CNO 99
#define NEO6M_TOP_CNO 4

struct neo6m {
  void *ctx;
  neo6m_uart_read_fn uart_read;
  char sentence[NEO6M_SENTENCE_MAX_CHARS + 1];
  uint8_t length;
  bool receiving;
  neo6m_data_t data;
  neo6m_stats_t stats;
  uint8_t gsv_next;
  uint8_t gsv_top_cno[NEO6M_TOP_CNO];
};

static neo6m_t instance;
static int instance_used = 0;

neo6m_t *neo6m_create(void *ctx, neo6m_uart_read_fn uart_read) {
  if (uart_read == NULL) {
    return NULL;
  }
  if (instance_used) {
    return NULL;
  }
  memset(&instance, 0, sizeof(instance));
  instance.ctx = ctx;
  instance.uart_read = uart_read;
  instance_used = 1;
  return &instance;
}

static bool is_digit(char c) { return c >= '0' && c <= '9'; }

static int hex_value(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

static bool parse_fixed(const char *s, uint8_t decimals, uint32_t *out) {
  uint32_t value = 0;
  uint8_t fraction_digits = 0;
  bool seen_point = false;
  bool seen_digit = false;
  for (; *s != '\0'; s++) {
    if (*s == '.') {
      if (seen_point) {
        return false;
      }
      seen_point = true;
      continue;
    }
    if (!is_digit(*s)) {
      return false;
    }
    seen_digit = true;
    if (seen_point) {
      if (fraction_digits == decimals) {
        continue;
      }
      fraction_digits++;
    }
    uint32_t digit = (uint32_t)(*s - '0');
    if (value > (UINT32_MAX - digit) / 10u) {
      return false;
    }
    value = value * 10u + digit;
  }
  if (!seen_digit) {
    return false;
  }
  for (; fraction_digits < decimals; fraction_digits++) {
    if (value > UINT32_MAX / 10u) {
      return false;
    }
    value *= 10u;
  }
  *out = value;
  return true;
}

static bool parse_signed_fixed(const char *s, uint8_t decimals, int32_t *out) {
  bool negative = (*s == '-');
  if (negative) {
    s++;
  }
  uint32_t magnitude;
  if (!parse_fixed(s, decimals, &magnitude) || magnitude > INT32_MAX) {
    return false;
  }
  *out = negative ? -(int32_t)magnitude : (int32_t)magnitude;
  return true;
}

static bool parse_two_digits(const char *s, uint8_t *out) {
  if (!is_digit(s[0]) || !is_digit(s[1])) {
    return false;
  }
  *out = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
  return true;
}

static bool parse_time(const char *s, uint8_t *hours, uint8_t *minutes,
                       uint8_t *seconds) {
  uint8_t h, m, sec;
  if (strlen(s) < 6 || !parse_two_digits(s, &h) ||
      !parse_two_digits(s + 2, &m) || !parse_two_digits(s + 4, &sec)) {
    return false;
  }
  if (h > 23 || m > 59 || sec > 60) {
    return false;
  }
  *hours = h;
  *minutes = m;
  *seconds = sec;
  return true;
}

static bool parse_date(const char *s, uint8_t *day, uint8_t *month,
                       uint8_t *year) {
  uint8_t d, m, y;
  if (strlen(s) != 6 || !parse_two_digits(s, &d) ||
      !parse_two_digits(s + 2, &m) || !parse_two_digits(s + 4, &y)) {
    return false;
  }
  if (d < 1 || d > 31 || m < 1 || m > 12) {
    return false;
  }
  *day = d;
  *month = m;
  *year = y;
  return true;
}

static bool parse_coordinate(const char *value, const char *hemisphere,
                             char positive, char negative, uint32_t max_degrees,
                             int32_t *out_e7) {
  if (hemisphere[0] == '\0' || hemisphere[1] != '\0') {
    return false;
  }
  if (hemisphere[0] != positive && hemisphere[0] != negative) {
    return false;
  }
  uint32_t raw;
  if (!parse_fixed(value, NEO6M_MINUTES_DECIMALS, &raw)) {
    return false;
  }
  uint32_t degrees = raw / (100u * NEO6M_MINUTES_SCALE);
  uint32_t minutes_e5 = raw % (100u * NEO6M_MINUTES_SCALE);
  if (degrees > max_degrees || minutes_e5 >= 60u * NEO6M_MINUTES_SCALE) {
    return false;
  }
  uint32_t e7 = degrees * NEO6M_DEGREES_SCALE + (minutes_e5 * 100u) / 60u;
  if (e7 > max_degrees * NEO6M_DEGREES_SCALE) {
    return false;
  }
  *out_e7 = (hemisphere[0] == negative) ? -(int32_t)e7 : (int32_t)e7;
  return true;
}

static bool verify_and_strip_checksum(char *sentence) {
  char *star = strchr(sentence, '*');
  if (star == NULL || strlen(star + 1) != NEO6M_CHECKSUM_DIGITS) {
    return false;
  }
  int high = hex_value(star[1]);
  int low = hex_value(star[2]);
  if (high < 0 || low < 0) {
    return false;
  }
  uint8_t checksum = 0;
  for (const char *p = sentence; p < star; p++) {
    checksum ^= (uint8_t)*p;
  }
  *star = '\0';
  return checksum == (uint8_t)((high << 4) | low);
}

static uint8_t split_fields(char *sentence, char *fields[],
                            uint8_t max_fields) {
  uint8_t count = 0;
  fields[count++] = sentence;
  for (char *p = sentence; *p != '\0'; p++) {
    if (*p == ',') {
      *p = '\0';
      if (count == max_fields) {
        break;
      }
      fields[count++] = p + 1;
    }
  }
  return count;
}

static bool sentence_is(const char *address, const char *formatter) {
  return strlen(address) == NEO6M_ADDRESS_LENGTH &&
         strcmp(address + 2, formatter) == 0;
}

static bool handle_rmc(neo6m_t *dev, char *fields[], uint8_t count) {
  if (count <= RMC_DATE) {
    return false;
  }
  neo6m_data_t *d = &dev->data;
  d->fix_valid = (fields[RMC_STATUS][0] == 'A');
  d->time_valid =
      parse_time(fields[RMC_TIME], &d->hours, &d->minutes, &d->seconds);
  d->date_valid = parse_date(fields[RMC_DATE], &d->day, &d->month, &d->year);

  int32_t latitude, longitude;
  d->position_valid =
      parse_coordinate(fields[RMC_LATITUDE], fields[RMC_NS], 'N', 'S',
                       NEO6M_MAX_LATITUDE, &latitude) &&
      parse_coordinate(fields[RMC_LONGITUDE], fields[RMC_EW], 'E', 'W',
                       NEO6M_MAX_LONGITUDE, &longitude);
  if (d->position_valid) {
    d->latitude_e7 = latitude;
    d->longitude_e7 = longitude;
  }

  uint32_t knots_x1000;
  d->speed_valid = parse_fixed(fields[RMC_SPEED], 3, &knots_x1000);
  if (d->speed_valid) {
    d->speed_kmh_x10 = (uint32_t)((uint64_t)knots_x1000 *
                                  NEO6M_METERS_PER_NAUTICAL_MILE / 100000u);
  }

  uint32_t course_x100;
  d->course_valid = parse_fixed(fields[RMC_COURSE], 2, &course_x100) &&
                    course_x100 < NEO6M_MAX_COURSE_X100;
  if (d->course_valid) {
    d->course_x100 = (uint16_t)course_x100;
  }
  return true;
}

static bool handle_gga(neo6m_t *dev, char *fields[], uint8_t count) {
  if (count <= GGA_ALTITUDE) {
    return false;
  }
  neo6m_data_t *d = &dev->data;
  uint32_t value;
  d->fix_quality = (parse_fixed(fields[GGA_QUALITY], 0, &value) && value <= 9)
                       ? (uint8_t)value
                       : 0;
  d->satellites =
      (parse_fixed(fields[GGA_SATELLITES], 0, &value) && value <= UINT8_MAX)
          ? (uint8_t)value
          : 0;
  d->hdop_valid =
      parse_fixed(fields[GGA_HDOP], 2, &value) && value <= UINT16_MAX;
  if (d->hdop_valid) {
    d->hdop_x100 = (uint16_t)value;
  }
  d->altitude_valid =
      parse_signed_fixed(fields[GGA_ALTITUDE], 1, &d->altitude_dm);
  return true;
}

static void insert_top_cno(uint8_t top[], uint8_t value) {
  for (uint8_t i = 0; i < NEO6M_TOP_CNO; i++) {
    if (value > top[i]) {
      uint8_t smaller = top[i];
      top[i] = value;
      value = smaller;
    }
  }
}

static uint8_t average_top_cno(const uint8_t top[]) {
  uint16_t sum = 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < NEO6M_TOP_CNO; i++) {
    if (top[i] > 0) {
      sum += top[i];
      count++;
    }
  }
  return (count > 0) ? (uint8_t)(sum / count) : 0;
}

static bool handle_gsv(neo6m_t *dev, char *fields[], uint8_t count) {
  uint32_t messages, number, in_view;
  if (count <= GSV_IN_VIEW ||
      !parse_fixed(fields[GSV_MESSAGES], 0, &messages) ||
      !parse_fixed(fields[GSV_MESSAGE_NUMBER], 0, &number) ||
      !parse_fixed(fields[GSV_IN_VIEW], 0, &in_view) || messages == 0 ||
      number == 0 || number > messages || in_view > UINT8_MAX) {
    dev->gsv_next = 0;
    return false;
  }
  if (number == 1) {
    dev->gsv_next = 1;
    memset(dev->gsv_top_cno, 0, sizeof(dev->gsv_top_cno));
  }
  if (number != dev->gsv_next) {
    dev->gsv_next = 0;
    return false;
  }
  for (uint8_t i = GSV_FIRST_CNO; i < count; i += GSV_BLOCK_FIELDS) {
    uint32_t cno;
    if (parse_fixed(fields[i], 0, &cno) && cno > 0 && cno <= NEO6M_MAX_CNO) {
      insert_top_cno(dev->gsv_top_cno, (uint8_t)cno);
    }
  }
  if (number < messages) {
    dev->gsv_next++;
    return false;
  }
  dev->gsv_next = 0;
  dev->data.sky_valid = true;
  dev->data.satellites_in_view = (uint8_t)in_view;
  dev->data.cno_top4_avg = average_top_cno(dev->gsv_top_cno);
  return true;
}

static uint32_t handle_sentence(neo6m_t *dev) {
  if (!verify_and_strip_checksum(dev->sentence)) {
    dev->stats.checksum_errors++;
    return NEO6M_UPDATED_NONE;
  }
  dev->stats.sentences++;

  char *fields[NEO6M_MAX_FIELDS];
  uint8_t count = split_fields(dev->sentence, fields, NEO6M_MAX_FIELDS);
  if (sentence_is(fields[0], "RMC")) {
    return handle_rmc(dev, fields, count) ? NEO6M_UPDATED_RMC
                                          : NEO6M_UPDATED_NONE;
  }
  if (sentence_is(fields[0], "GGA")) {
    return handle_gga(dev, fields, count) ? NEO6M_UPDATED_GGA
                                          : NEO6M_UPDATED_NONE;
  }
  if (sentence_is(fields[0], "GSV")) {
    return handle_gsv(dev, fields, count) ? NEO6M_UPDATED_GSV
                                          : NEO6M_UPDATED_NONE;
  }
  return NEO6M_UPDATED_NONE;
}

static uint32_t feed_byte(neo6m_t *dev, char c) {
  if (c == '$') {
    dev->length = 0;
    dev->receiving = true;
    return NEO6M_UPDATED_NONE;
  }
  if (!dev->receiving) {
    return NEO6M_UPDATED_NONE;
  }
  if (c == '\r' || c == '\n') {
    dev->receiving = false;
    dev->sentence[dev->length] = '\0';
    return handle_sentence(dev);
  }
  if (dev->length >= NEO6M_SENTENCE_MAX_CHARS) {
    dev->receiving = false;
    dev->stats.overflows++;
    return NEO6M_UPDATED_NONE;
  }
  dev->sentence[dev->length++] = c;
  return NEO6M_UPDATED_NONE;
}

neo6m_status_t neo6m_process(neo6m_t *dev, uint32_t *updated) {
  if (dev == NULL || updated == NULL) {
    return NEO6M_NULL_PTR;
  }
  *updated = NEO6M_UPDATED_NONE;
  uint8_t chunk[NEO6M_READ_CHUNK];
  int count;
  while ((count = dev->uart_read(dev->ctx, chunk, sizeof(chunk))) > 0) {
    for (int i = 0; i < count; i++) {
      *updated |= feed_byte(dev, (char)chunk[i]);
    }
  }
  return (count < 0) ? NEO6M_UART_ERROR : NEO6M_OK;
}

neo6m_status_t neo6m_get_data(const neo6m_t *dev, neo6m_data_t *data) {
  if (dev == NULL || data == NULL) {
    return NEO6M_NULL_PTR;
  }
  *data = dev->data;
  return NEO6M_OK;
}

neo6m_status_t neo6m_get_stats(const neo6m_t *dev, neo6m_stats_t *stats) {
  if (dev == NULL || stats == NULL) {
    return NEO6M_NULL_PTR;
  }
  *stats = dev->stats;
  return NEO6M_OK;
}