#include "sun.h"
#include <math.h>

#define PI 3.14159265358979323846
#define RAD(x) ((x) * (PI / 180.0))
#define DEG(x) ((x) * (180.0 / PI))
#define JD_2000_EPOCH 2451544.5
#define SECONDS_PER_DAY 86400.0

typedef struct {
  double declination_rad;
  double equation_of_time_min;
} sun_state_t;

static double wrap360(double x) {
  x = fmod(x, 360.0);
  return (x < 0.0) ? x + 360.0 : x;
}

static void sun_state(int64_t t_s, sun_state_t *s) {
  double jd = JD_2000_EPOCH + (double)t_s / SECONDS_PER_DAY;
  double t = (jd - 2451545.0) / 36525.0;
  double l0 = wrap360(280.46646 + t * (36000.76983 + t * 0.0003032));
  double m = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  double c = sin(RAD(m)) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
             sin(RAD(2.0 * m)) * (0.019993 - 0.000101 * t) +
             sin(RAD(3.0 * m)) * 0.000289;
  double true_long = l0 + c;
  double omega = 125.04 - 1934.136 * t;
  double app_long = true_long - 0.00569 - 0.00478 * sin(RAD(omega));
  double mean_obliq =
      23.0 +
      (26.0 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0) /
          60.0;
  double obliq = mean_obliq + 0.00256 * cos(RAD(omega));
  s->declination_rad = asin(sin(RAD(obliq)) * sin(RAD(app_long)));
  double y = tan(RAD(obliq / 2.0));
  y *= y;
  double l0r = RAD(l0);
  double mr = RAD(m);
  s->equation_of_time_min =
      4.0 * DEG(y * sin(2.0 * l0r) - 2.0 * e * sin(mr) +
                4.0 * e * y * sin(mr) * cos(2.0 * l0r) -
                0.5 * y * y * sin(4.0 * l0r) - 1.25 * e * e * sin(2.0 * mr));
}

static int64_t day_start(int64_t t_s) {
  int64_t day = t_s / 86400;
  if (t_s < 0 && t_s % 86400 != 0) {
    day--;
  }
  return day * 86400;
}

static int64_t solar_noon(int64_t t_s, double lon_deg, const sun_state_t *s) {
  double minutes = 720.0 - 4.0 * lon_deg - s->equation_of_time_min;
  return day_start(t_s) + (int64_t)llround(minutes * 60.0);
}

static sun_event_status_t hour_angle(double lat_deg, const sun_state_t *s,
                                     double zenith_deg, double *ha_deg) {
  double lat = RAD(lat_deg);
  double x = cos(RAD(zenith_deg)) / (cos(lat) * cos(s->declination_rad)) -
             tan(lat) * tan(s->declination_rad);
  if (x < -1.0) {
    return SUN_EVENT_ALWAYS_UP;
  }
  if (x > 1.0) {
    return SUN_EVENT_ALWAYS_DOWN;
  }
  *ha_deg = DEG(acos(x));
  return SUN_EVENT_OK;
}

static sun_event_status_t event_time(int64_t noon_s, double lat_deg,
                                     double lon_deg, double zenith_deg,
                                     int sign, int64_t *event_s) {
  int64_t t = noon_s;
  sun_event_status_t status = SUN_EVENT_OK;
  for (int pass = 0; pass < 2; pass++) {
    sun_state_t s;
    sun_state(t, &s);
    double ha;
    status = hour_angle(lat_deg, &s, zenith_deg, &ha);
    if (status != SUN_EVENT_OK) {
      return status;
    }
    int64_t noon = solar_noon(noon_s, lon_deg, &s);
    t = noon + sign * (int64_t)llround(ha * 4.0 * 60.0);
  }
  *event_s = t;
  return SUN_EVENT_OK;
}

void sun_day(int64_t ref_s, double lat_deg, double lon_deg, sun_day_t *day) {
  sun_state_t s;
  sun_state(ref_s, &s);
  int64_t noon = solar_noon(ref_s, lon_deg, &s);
  if (noon - ref_s > 43200) {
    noon -= 86400;
  } else if (ref_s - noon > 43200) {
    noon += 86400;
  }
  sun_state(noon, &s);
  int64_t refined = solar_noon(noon, lon_deg, &s);
  day->noon_s = refined;

  day->status = event_time(refined, lat_deg, lon_deg, SUN_ZENITH_SUNRISE_DEG,
                           -1, &day->rise_s);
  if (day->status == SUN_EVENT_OK) {
    day->status = event_time(refined, lat_deg, lon_deg,
                             SUN_ZENITH_SUNRISE_DEG, 1, &day->set_s);
  }
  day->civil_status = event_time(refined, lat_deg, lon_deg,
                                 SUN_ZENITH_CIVIL_DEG, -1, &day->dawn_s);
  if (day->civil_status == SUN_EVENT_OK) {
    day->civil_status = event_time(refined, lat_deg, lon_deg,
                                   SUN_ZENITH_CIVIL_DEG, 1, &day->dusk_s);
  }
}

static double refraction_deg(double elevation_deg) {
  if (elevation_deg > 85.0) {
    return 0.0;
  }
  double te = tan(RAD(elevation_deg));
  double arcsec;
  if (elevation_deg > 5.0) {
    arcsec = 58.1 / te - 0.07 / (te * te * te) + 0.000086 / pow(te, 5.0);
  } else if (elevation_deg > -0.575) {
    double e = elevation_deg;
    arcsec = 1735.0 + e * (-518.2 + e * (103.4 + e * (-12.79 + e * 0.711)));
  } else {
    arcsec = -20.772 / te;
  }
  return arcsec / 3600.0;
}

void sun_position(int64_t t_s, double lat_deg, double lon_deg,
                  double *elevation_deg, double *azimuth_deg) {
  sun_state_t s;
  sun_state(t_s, &s);
  double minutes = (double)(t_s - day_start(t_s)) / 60.0;
  double true_solar = fmod(minutes + s.equation_of_time_min + 4.0 * lon_deg,
                           1440.0);
  if (true_solar < 0.0) {
    true_solar += 1440.0;
  }
  double ha = (true_solar / 4.0 < 0.0) ? true_solar / 4.0 + 180.0
                                       : true_solar / 4.0 - 180.0;
  double lat = RAD(lat_deg);
  double cos_zenith = sin(lat) * sin(s.declination_rad) +
                      cos(lat) * cos(s.declination_rad) * cos(RAD(ha));
  if (cos_zenith > 1.0) {
    cos_zenith = 1.0;
  } else if (cos_zenith < -1.0) {
    cos_zenith = -1.0;
  }
  double zenith = acos(cos_zenith);
  double elevation = 90.0 - DEG(zenith);
  *elevation_deg = elevation + refraction_deg(elevation);

  double denominator = cos(lat) * sin(zenith);
  double azimuth;
  if (fabs(denominator) < 1e-9) {
    azimuth = (lat_deg > 0.0) ? 180.0 : 0.0;
  } else {
    double x = (sin(lat) * cos(zenith) - sin(s.declination_rad)) / denominator;
    if (x > 1.0) {
      x = 1.0;
    } else if (x < -1.0) {
      x = -1.0;
    }
    double a = DEG(acos(x));
    azimuth = (ha > 0.0) ? wrap360(a + 180.0) : wrap360(540.0 - a);
  }
  *azimuth_deg = azimuth;
}
