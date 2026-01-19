#pragma once
#include <stdint.h>

void drawStartupScreen(void);
void drawSensorBar(uint32_t min, uint32_t max, uint32_t cur);
void drawMeasurement(uint32_t baseline, uint32_t new, uint32_t latency);
void printAverage(float mean_ms, float sd_ms);