#pragma once
#include <stdint.h>
#include "main.h"

#define MS_FACTOR 1000
#define MEASUREMENT_DELAY 503

uint32_t readADC();
uint32_t readAveragedADC();
void measure();
void computeStatsMs(float *mean_ms, float *sd_ms);
