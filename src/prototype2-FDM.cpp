
/*
 * Three-Wavelength FDM Oximeter
 * Target: Particle Argon (nRF52840)
 *
 * LEDs:
 *   660 nm -> D8 -> 7 Hz
 *   805 nm -> A4 -> 11 Hz
 *   940 nm -> A0 -> 17 Hz
 *
 * Detector: OPT101P -> A1
 *
 * ADC:
 *   20,000 samples/second
 *   12-bit resolution
 *   Hardware timer + DMA
 *   Double-buffered acquisition
 *
 * Demodulation:
 *   Three simultaneous I/Q correlations
 *   1-second measurement windows
 *
 * CSV:
 * dc_v,red_v,ir805_v,ir940_v,
 * dropped_buffers,invalid_windows
 */

#include "Particle.h"
#include "ADCDMAGen3_RK.h"
#include <math.h>

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

// ----------------------------
// LED configuration
// ----------------------------

const int LED_PINS[3] = {D8, A4, A0};

// Placeholder modulation frequencies
const uint32_t LED_FREQS[3] = {7, 11, 17};

const uint8_t DUTY_CYCLE = 128;

// ----------------------------
// ADC configuration
// ----------------------------

const int SENSOR_PIN = A1;

// Change to 10000 for 10 kHz
const uint32_t SAMPLE_RATE = 20000;

// 2000 samples = 100 ms per buffer
// at 20 kHz.
const size_t BUFFER_SIZE = 2000;

// 1-second demodulation window
const uint32_t WINDOW_SIZE = SAMPLE_RATE;

// ADC: 12-bit, nominally 0-3.3 V
const float ADC_SCALE = 3.3f / 4095.0f;

const float TWO_PI = 6.28318530718f;

// ----------------------------
// DMA configuration
// ----------------------------

ADCDMAGen3 adc;

// DMA buffers (8 KB total)
static nrf_saadc_value_t buffer0[BUFFER_SIZE];
static nrf_saadc_value_t buffer1[BUFFER_SIZE];

// Shared between interrupt and loop
static nrf_saadc_value_t *volatile
    readyBuffer = nullptr;

static volatile uint32_t readySequence = 0;
static volatile uint32_t callbackSequence = 0;

// Main-loop counters
uint32_t lastSequence = 0;
uint32_t droppedBuffers = 0;
uint32_t invalidWindows = 0;

// ----------------------------
// Demodulation
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

uint32_t sampleCount = 0;
float dcSum = 0.0f;

// ----------------------------
// Output state
// ----------------------------

struct Measurement {
    float dc;
    float amplitude[3];

    uint32_t droppedBuffers;
    uint32_t invalidWindows;
};

Measurement latestMeasurement;

bool measurementReady = false;
bool headerPrinted = false;
bool adcStarted = false;

ret_code_t adcError = 0;

// ----------------------------
// Reset measurement window
// ----------------------------

void resetWindow() {
    sampleCount = 0;
    dcSum = 0.0f;

    for (int i = 0; i < 3; i++) {
        Channel &ch = channels[i];

        ch.cosPhase = 1.0f;
        ch.sinPhase = 0.0f;

        ch.sumI = 0.0f;
        ch.sumQ = 0.0f;
    }
}

// ----------------------------
// Finish measurement window
// ----------------------------

void finishWindow() {
    Measurement &m = latestMeasurement;

    m.dc = (dcSum / WINDOW_SIZE) * ADC_SCALE;

    for (int i = 0; i < 3; i++) {
        Channel &ch = channels[i];

        float I = ch.sumI;
        float Q = ch.sumQ;

        float magnitude = sqrtf(I * I + Q * Q);

        m.amplitude[i] =
            (2.0f / WINDOW_SIZE) *
            magnitude * ADC_SCALE;
    }

    m.droppedBuffers = droppedBuffers;
    m.invalidWindows = invalidWindows;

    measurementReady = true;

    resetWindow();
}

// ----------------------------
// Process individual sample
// ----------------------------

void processSample(int16_t raw) {
    float sample = (float)raw;

    dcSum += sample;

    for (int i = 0; i < 3; i++) {
        Channel &ch = channels[i];

        // In-phase and quadrature sums
        ch.sumI += sample * ch.cosPhase;
        ch.sumQ += sample * ch.sinPhase;

        // Update reference oscillator
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

    if (sampleCount >= WINDOW_SIZE) {
        finishWindow();
    }
}

// ----------------------------
// Process completed DMA buffer
// ----------------------------

void processBuffer(
    nrf_saadc_value_t *buffer,
    size_t size
) {
    for (size_t i = 0; i < size; i++) {
        processSample(buffer[i]);
    }
}

// ----------------------------
// DMA callback
// Runs in interrupt context
// ----------------------------

void onBufferReady(
    nrf_saadc_value_t *buffer,
    size_t size
) {
    // No processing or serial output here.
    // Publish the newest completed buffer.
    ++callbackSequence;

    readySequence = callbackSequence;
    readyBuffer = buffer;
}

// ----------------------------
// Serial output
// ----------------------------

void printMeasurement() {
    if (!Serial.isConnected()) {
        return;
    }

    Measurement &m = latestMeasurement;

    Serial.print(m.dc, 6);

    for (int i = 0; i < 3; i++) {
        Serial.print(",");
        Serial.print(m.amplitude[i], 6);
    }

    Serial.print(",");
    Serial.print(m.droppedBuffers);

    Serial.print(",");
    Serial.println(m.invalidWindows);
}

// ----------------------------
// Setup
// ----------------------------

void setup() {
    Serial.begin(115200);

    // Configure hardware PWM
    for (int i = 0; i < 3; i++) {
        pinMode(LED_PINS[i], OUTPUT);

        analogWrite(
            LED_PINS[i],
            DUTY_CYCLE,
            LED_FREQS[i]
        );

        // Reference oscillator increments
        float angle =
            TWO_PI * LED_FREQS[i] / SAMPLE_RATE;

        channels[i].cosStep = cosf(angle);
        channels[i].sinStep = sinf(angle);
    }

    resetWindow();

    // Configure continuous ADC DMA
    adcError = adc
        .withSampleFreqHz(SAMPLE_RATE)
        .withDoubleBuffer(
            BUFFER_SIZE,
            buffer0,
            buffer1
        )
        .withSamplePin(SENSOR_PIN)
        .withResolution(NRF_SAADC_RESOLUTION_12BIT)
        .withAcqTime(NRF_SAADC_ACQTIME_10US)
        .withBufferCallback(onBufferReady)
        .init();

    if (adcError != 0) {
        return;
    }

    // Start hardware-timed acquisition
    adc.start();
    adcStarted = true;
}

// ----------------------------
// Main loop
// ----------------------------

void loop() {
    // Handle serial connection
    if (Serial.isConnected()) {
        if (!headerPrinted) {
            if (!adcStarted) {
                Serial.print("ADC initialization failed: ");
                Serial.println((unsigned long)adcError);
            } else {
                Serial.println(
                    "dc_v,red_v,ir805_v,ir940_v,"
                    "dropped_buffers,invalid_windows"
                );
            }

            headerPrinted = true;
        }
    } else {
        headerPrinted = false;
    }

    if (!adcStarted) {
        return;
    }

    // Retrieve completed DMA buffer.
    // Keep interrupt masking very short.
    nrf_saadc_value_t *buffer;
    uint32_t sequence;

    noInterrupts();

    buffer = readyBuffer;
    sequence = readySequence;
    readyBuffer = nullptr;

    interrupts();

    if (buffer != nullptr) {

        // Detect skipped buffers
        uint32_t expected = lastSequence + 1;

        if (sequence != expected) {
            droppedBuffers +=
                sequence - expected;

            // Discard incomplete measurement.
            // A gap makes the correlation invalid.
            if (sampleCount != 0) {
                invalidWindows++;
            }

            resetWindow();
        }

        lastSequence = sequence;

        // Process samples while DMA fills
        // the other buffer.
        processBuffer(buffer, BUFFER_SIZE);
    }

    // Serial output is outside both the
    // DMA callback and the sample loop.
    if (measurementReady && Serial.isConnected()) {
        printMeasurement();
        measurementReady = false;
    }
}
