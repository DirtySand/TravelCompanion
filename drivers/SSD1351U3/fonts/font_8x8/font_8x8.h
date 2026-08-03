#ifndef FONT_8X8_H
#define FONT_8X8_H
 
#include <stdint.h>
 
#define FONT_FIRST_CHAR   0x20
#define FONT_LAST_CHAR    0x7F
#define FONT_GLYPH_COUNT  (FONT_LAST_CHAR - FONT_FIRST_CHAR + 1)
#define FONT_DEGREE       ((char)0x7F)
 
extern const uint8_t font8x8[FONT_GLYPH_COUNT][8];

static inline const uint8_t *font8x8_glyph(char c) {
    unsigned char uc = (unsigned char)c;
    if (uc < FONT_FIRST_CHAR || uc > FONT_LAST_CHAR) uc = FONT_FIRST_CHAR;
    return font8x8[uc - FONT_FIRST_CHAR];
}
 
#endif