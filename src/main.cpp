#include "main.h"

uint32_t latencies_us[NUM_CYCLES] = {};
uint8_t cycle_index = 0;

uint16_t min_sensor_value = 65535;
uint16_t max_sensor_value = 0;

unsigned long start_millis;

volatile boolean startRequested = false;
volatile boolean restartRequested = false;
volatile boolean running = false;

/// @brief Inits analog pin and mouse HID
void setup() {
    initScreen();
    ADCSRA = (ADCSRA & 0xf8) | 0x04;

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr, FALLING);

    // first ADC read is wonky, discard
    analogRead(SENSOR_PIN);
    start_millis = millis();
}

/// @brief Measures brightness, waits for brightness change, saves latency. Shows an average after last cycle.
void loop() {
    yield();
    unsigned long current_millis = millis();

    if (startRequested) {
        startRequested = false;
        restartRequested = false;
        Mouse.begin();

        while (cycle_index < NUM_CYCLES) {
            measure();

            if (restartRequested) {
                Mouse.release();
                cycle_index = 0;

                drawInterrupted();
                delay(1000);
                startRequested = false;
                restartRequested = false;
                running = false;
                break;
            }
        }

        if (cycle_index == NUM_CYCLES) {
            double mean_ms = 0.0;
            double sd_ms = 0.0;
            computeStatsMs(&mean_ms, &sd_ms);
            printAverage(mean_ms, sd_ms);

            cycle_index = 0;
            running = false;
            startRequested = false;
            restartRequested = false;
            Mouse.end();

            while(digitalRead(BUTTON_PIN)){
              // small debounce
              delay(100);
            }
        }
    }

    if (current_millis - start_millis >= 50UL) {
        uint16_t brightness = analogRead(SENSOR_PIN);
        if (max_sensor_value < brightness) {
            max_sensor_value = brightness;
        }
        if (min_sensor_value > brightness) {
            min_sensor_value = brightness;
        }

        drawStartupScreen(brightness, min_sensor_value, max_sensor_value);
    }
}

void measure() {
    // get reference brightness
    const uint16_t baseline = analogRead(SENSOR_PIN);
    printMeasurement(baseline, cycle_index);

    running = true;

    // reset timer, click mouse
    const unsigned long start = micros();
    Mouse.move(0, 127, 0);

    while (true) {
        if (restartRequested) {
            return;
        }

        const int delta = analogRead(SENSOR_PIN) - baseline;

        // loop until brightness delta is bigger than threshold
        if (abs(delta) > BRIGHTNESS_THRESHOLD) {
            // save and sum measured latency
            unsigned long latency = micros() - start;
            Mouse.move(0, -127, 0);

            if (latency <= internalLatency) {
                printError();
                delay(1000);
                break;
            }

            latency = latency - internalLatency;

            // store cycle in the array
            if (cycle_index < NUM_CYCLES) {
                latencies_us[cycle_index] = latency;
            }

            printMeasurement(baseline, cycle_index, latency / MS_FACTOR, baseline + delta);

            delay(MEASUREMENT_DELAY_MS);

            cycle_index++;
            break;
        }
    }
}

void isr() {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 200) {
        (running ? restartRequested : startRequested) = true;
    }
    last_interrupt_time = interrupt_time;
}

/// @brief Calculates mean latency and sample standard deviation for an array of us latencies.
void computeStatsMs(double *mean_ms, double *sd_ms) {
    double sum_us = 0.0;
    double sd_us = 0.0;
    double mean_us = 0.0;
    double variance_us = 0.0;

    if (NUM_CYCLES == 1) {
        *mean_ms = static_cast<double>(latencies_us[0]) / MS_FACTOR;
        *sd_ms = 0.0;
        return;
    }

    // calculate mean
    for (const unsigned long latency: latencies_us) {
        sum_us += static_cast<double>(latency);
    }
    mean_us = sum_us / static_cast<double>(NUM_CYCLES);

    // calculate sample standard deviation
    for (const unsigned long latency: latencies_us) {
        const double diff_us = static_cast<double>(latency) - mean_us;
        variance_us += diff_us * diff_us;
    }
    sd_us = sqrt(variance_us / (NUM_CYCLES - 1));

    *mean_ms = mean_us / MS_FACTOR;
    *sd_ms = sd_us / MS_FACTOR;
}
