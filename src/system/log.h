#ifndef LOG_H
#define LOG_H

#include "board_config.h"
#include <stdio.h>

#if LOG_ENABLED
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...)                                                               \
  do {                                                                         \
    if (0) {                                                                   \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  } while (0)
#endif

#endif // LOG_H
