
/*
 * Three-Wavelength FDM Oximeter
 * Particle Argon + Teleplot
 *
 * LEDs:
 * 660 nm -> D8 -> 7 Hz
 * 805 nm -> A4 -> 11 Hz
 * 940 nm -> A0 -> 17 Hz
 *
 * OPT101P photodetector -> A1
 *
 * ADC sampling: 500 Hz
 * Raw Teleplot updates: 100 Hz
 * Demodulated updates: 1 Hz
 */

#include "Particle.h"
#include <math.h>
#include <stdio.h>

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

// ----------------------------
// Hardware configuration
// ----------------------------

const int LED_PINS[3] = {D8, A4, A0};
const uint32_t LED_FREQS[3] = {7, 11, 17};
const uint8_t DUTY_CYCLE = 128;

const int SENSOR_PIN = A1;

// ----------------------------
// Sampling configuration
// ----------------------------

const uint32_t SAMPLE_RATE = 10000;
const uint32_t SAMPLE_PERIOD_US = 100;
const uint16_t WINDOW_SIZE = SAMPLE_RATE;

// Send every 5th sample (100 Hz)
// Change to 10 for 50 Hz plotting
const uint8_t PLOT_EVERY_N_SAMPLES = 5;

const float ADC_SCALE = 3.3f / 4095.0f;
const float TWO_PI = 6.28318530718f;

// ----------------------------
// Demodulation state
// ----------------------------

struct Channel {
    float cosStep;
    float sinStep;
    float cosPhase;
    float sinPhase;
    float sumI;
    float sumQ;
};

Channel channels[3];

uint32_t nextSampleTime;
uint32_t droppedWindows = 0;

uint16_t sampleCount = 0;
float dcSum = 0;

// ----------------------------
// Teleplot state
// ----------------------------

// Latest raw sample for plotting
float plotVoltage = 0;
bool plotPending = false;
uint8_t plotDivider = 0;

// Most recent demodulation results
float lastDC = 0;
float lastAmplitude[3] = {0, 0, 0};
uint32_t lastDropped = 0;

// 0-4: summary channels waiting to send
// 5: no summary pending
uint8_t summaryIndex = 5;

// ----------------------------
// Reset measurement window
// ----------------------------

void resetWindow() {
    sampleCount = 0;
    dcSum = 0;

    for (int i = 0; i < 3; i++) {
        channels[i].cosPhase = 1.0f;
        channels[i].sinPhase = 0.0f;
        channels[i].sumI = 0.0f;
        channels[i].sumQ = 0.0f;
    }
}

// ----------------------------
// Process ADC sample
// ----------------------------

void processSample(float voltage) {
    for (int i = 0; i < 3; i++) {
        Channel &ch = channels[i];

        ch.sumI += voltage * ch.cosPhase;
        ch.sumQ += voltage * ch.sinPhase;

        float nextCos =
            ch.cosPhase * ch.cosStep -
            ch.sinPhase * ch.sinStep;

        float nextSin =
            ch.sinPhase * ch.cosStep +
            ch.cosPhase * ch.sinStep;

        ch.cosPhase = nextCos;
        ch.sinPhase = nextSin;
    }

    sampleCount++;
}

// ----------------------------
// Calculate completed window
// ----------------------------

void reportWindow() {
    lastDC = dcSum / WINDOW_SIZE;

    for (int i = 0; i < 3; i++) {
        float I = channels[i].sumI;
        float Q = channels[i].sumQ;

        lastAmplitude[i] =
            (2.0f / WINDOW_SIZE) *
            sqrtf(I * I + Q * Q);
    }

    lastDropped = droppedWindows;

    // Queue results for Teleplot
    summaryIndex = 0;
}

// ----------------------------
// Setup
// ----------------------------

void setup() {
    Serial.begin(115200);

    pinMode(SENSOR_PIN, INPUT);

    // Configure LED modulation
    for (int i = 0; i < 3; i++) {
        pinMode(LED_PINS[i], OUTPUT);

        analogWrite(
            LED_PINS[i],
            DUTY_CYCLE,
            LED_FREQS[i]
        );

        float angle =
            TWO_PI * LED_FREQS[i] /
            SAMPLE_RATE;

        channels[i].cosStep = cosf(angle);
        channels[i].sinStep = sinf(angle);
    }

    resetWindow();

    nextSampleTime =
        micros() + SAMPLE_PERIOD_US;
}

// ----------------------------
// Main loop
// ----------------------------

void loop() {
    uint32_t now = micros();

    int32_t lateness =
        (int32_t)(now - nextSampleTime);

    // Between ADC samples, send plot data
    if (lateness < 0) {
        // Allow at least 1.2 ms before
        // the next scheduled ADC sample
        if (-lateness > 1200) {
            ;
        }
        return;
    }

    // Detect missed sampling deadlines
    if (lateness >=
        (int32_t)SAMPLE_PERIOD_US) {

        droppedWindows++;
        resetWindow();

        return;
    }

    nextSampleTime += SAMPLE_PERIOD_US;

    // Read photodetector
    int raw = analogRead(SENSOR_PIN);
    float voltage = raw * ADC_SCALE;

    processSample(voltage);

    // Completed 1-second window
    if (sampleCount >= WINDOW_SIZE) {
        reportWindow();
        resetWindow();

        nextSampleTime =
            micros() + SAMPLE_PERIOD_US;
    }
}