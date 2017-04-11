
#ifndef __DECLINATION_H_
#define __DECLINATION_H_
#include "stdint.h"
#include "app_math.h"

int16_t get_lookup_value(uint8_t x, uint8_t y);
float get_declination(float lat, float lon);

#endif //declination
