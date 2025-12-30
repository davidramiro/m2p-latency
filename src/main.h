#pragma once

#include <Arduino.h>
#include "Mouse.h"
#include <math.h>
#include "display.h"

/// @brief Analog pin connected to signal of photodiode/photoresistor
static constexpr uint8_t SENSOR_PIN = A3;
/// @brief Pin shorted to ground via button
static constexpr uint8_t BUTTON_PIN = 7;
/// @brief RX LED PIN to show fault
static constexpr uint8_t RX_LED_PIN = 17;
/// @brief Sensor threshold for registering a screen change event in 40 mV increments of the analog readout.
static constexpr uint16_t BRIGHTNESS_THRESHOLD = 10;
/// @brief Number of measurements before calculating summary
static constexpr uint8_t NUM_CYCLES = 20;
/// @brief Internal latency of the analog read, this lag will be subtracted from the measured latency
static constexpr uint16_t internalLatency = 40;
/// @brief Delay between measurement cycles (should not be a divisor of display refresh rate)
static constexpr uint16_t MEASUREMENT_DELAY_MS = 574;
/// @brief Conversion factor between us and ms
static constexpr double MS_FACTOR = 1000.0;

void initScreen();

void isr();

void measure();

void computeStatsMs(double *mean_ms, double *sd_ms);
