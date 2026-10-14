
/*
 * Three-Wavelength FDM Oximeter
 * Target: Particle Argon
 *
 * LEDs: 660 nm (7 Hz), 805 nm (11 Hz), 940 nm (17 Hz)
 * Detector: OPT101P on A1
 *
 * Samples at 500 Hz and separates the three
 * frequencies using synchronous demodulation.
 *
 * Serial CSV output:
 * dc_v,red_v,ir805_v,ir940_v,dropped_windows
 */

#include "Particle.h"
#include <math.h>

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

// LED configuration
const int LED_PINS[3] = {D8, A4, A0};
const uint32_t LED_FREQS[3] = {7, 11, 17};
const uint8_t DUTY_CYCLE = 128;

// Photodetector configuration
const int SENSOR_PIN = A1;
const uint32_t SAMPLE_RATE = 500;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;
const uint16_t WINDOW_SIZE = SAMPLE_RATE;

// ADC: 12-bit, 0-3.3V
const float ADC_SCALE = 3.3f / 4095.0f;
const float TWO_PI = 6.28318530718f;

// Demodulation state for each wavelength
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
bool headerPrinted = false;

// Reset accumulators for a new measurement window
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

// Process one photodetector sample
void processSample(float voltage) {
    dcSum += voltage;

    for (int i = 0; i < 3; i++) {
        Channel &ch = channels[i];

        // In-phase and quadrature correlation
        ch.sumI += voltage * ch.cosPhase;
        ch.sumQ += voltage * ch.sinPhase;

        // Advance reference oscillator
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

// Output one completed measurement window
void reportWindow() {
    float dc = dcSum / WINDOW_SIZE;
    float amplitude[3];

    for (int i = 0; i < 3; i++) {
        float I = channels[i].sumI;
        float Q = channels[i].sumQ;

        amplitude[i] =
            (2.0f / WINDOW_SIZE) *
            sqrtf(I * I + Q * Q);
    }

    if (Serial.isConnected()) {
        Serial.print(dc, 6);

        for (int i = 0; i < 3; i++) {
            Serial.print(",");
            Serial.print(amplitude[i], 6);
        }

        Serial.print(",");
        Serial.println(droppedWindows);
    }
}

void setup() {
    Serial.begin(115200);

    // Configure and start independent PWM
    for (int i = 0; i < 3; i++) {
        pinMode(LED_PINS[i], OUTPUT);

        analogWrite(
            LED_PINS[i],
            DUTY_CYCLE,
            LED_FREQS[i]
        );

        // Precompute reference oscillator increments
        float angle =
            TWO_PI * LED_FREQS[i] / SAMPLE_RATE;

        channels[i].cosStep = cosf(angle);
        channels[i].sinStep = sinf(angle);
    }

    resetWindow();

    nextSampleTime = micros() + SAMPLE_PERIOD_US;
}

void loop() {
    // Print CSV header when serial connects
    if (Serial.isConnected()) {
        if (!headerPrinted) {
            Serial.println(
                "dc_v,red_v,ir805_v,ir940_v,dropped_windows"
            );
            headerPrinted = true;
        }
    } else {
        headerPrinted = false;
    }

    uint32_t now = micros();
    int32_t lateness =
        (int32_t)(now - nextSampleTime);

    // Wait until the next scheduled ADC sample
    if (lateness < 0) {
        return;
    }

    // Discard incomplete windows if sampling
    // falls more than one interval behind.
    if (lateness >= (int32_t)SAMPLE_PERIOD_US) {
        droppedWindows++;
        resetWindow();
        nextSampleTime = now + SAMPLE_PERIOD_US;
        return;
    }

    nextSampleTime += SAMPLE_PERIOD_US;

    // Read OPT101P
    int raw = analogRead(SENSOR_PIN);
    float voltage = raw * ADC_SCALE;

    processSample(voltage);

    // One complete measurement window
    if (sampleCount >= WINDOW_SIZE) {
        reportWindow();
        resetWindow();

        // Restart after reporting, avoiding
        // serial-output timing affecting samples.
        nextSampleTime = micros() + SAMPLE_PERIOD_US;
    }
}
