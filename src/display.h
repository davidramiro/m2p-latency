#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static constexpr uint8_t SCREEN_WIDTH = 128;
static constexpr uint8_t SCREEN_HEIGHT = 64;
static constexpr int16_t LOWER_CURSOR_Y = SCREEN_HEIGHT / 2 - 4;
static constexpr int16_t BOTTOM_CURSOR_Y = SCREEN_HEIGHT / 3 * 2;
static constexpr uint8_t GLYPH_NEW = 0x1A;
static constexpr uint8_t GLYPH_AVG = 0xE5;
static constexpr uint8_t GLYPH_SIGMA = 0xE4;
static constexpr uint8_t GLYPH_LBRAK = 0xAF;
static constexpr uint8_t GLYPH_RBRAK = 0xAE;
static constexpr uint8_t DISPLAY_I2C_ADDR = 0x3C;
static constexpr uint8_t BLINK_DELAY = 200;

void drawMsValue(double mean_ms);

void drawStdDevValue(double sd_ms);

void printMeasurement(uint16_t baseline, uint8_t cycle_index, double cycle_latency = 0.0, uint16_t measured = 0);

void printAverage(double mean_ms, double sd_ms);

void showHeader();

void printError();

void drawStartupScreen(uint16_t cur, uint16_t min, uint16_t max);

void drawInterrupted();
