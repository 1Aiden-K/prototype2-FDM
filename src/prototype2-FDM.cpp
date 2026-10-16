/*
 * Three-Wavelength FDM Oximeter
 * Particle Argon (nRF52840)
 *
 * ADC: A1, 20 kHz, 12-bit
 * LEDs: D8 (7 Hz), A4 (11 Hz), A0 (17 Hz)
 *
 * Uses SAADC internal timer + EasyDMA + PPI.
 * Requires no third-party libraries.
 */

#include "Particle.h"
#undef CHG
#include "nrf.h"
#include "nrfx_ppi.h"
#include <math.h>

SYSTEM_MODE(MANUAL);

// ----------------------------
// Configuration
// ----------------------------

const int LED_PINS[3] = {D8, A4, A0};
const uint32_t LED_FREQS[3] = {7, 11, 17};

const uint32_t SAMPLE_RATE = 20000;
const uint32_t BUFFER_SIZE = 2000;
const uint32_t BUFFER_COUNT = 4;
const uint32_t WINDOW_SIZE = SAMPLE_RATE;

const float ADC_SCALE = 3.3f / 4095.0f;
constexpr float TWO_PI_F = 6.28318530718f;

// ----------------------------
// ADC buffers and state
// ----------------------------

static int16_t adcBuffers[BUFFER_COUNT][BUFFER_SIZE];

volatile uint32_t completedBuffers = 0;

uint32_t processedBuffers = 0;
uint32_t droppedBuffers = 0;
uint32_t invalidWindows = 0;

nrf_ppi_channel_t ppiChannel;

bool adcRunning = false;
bool headerPrinted = false;

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

    float sumCos;
    float sumSin;
};

Channel channels[3];

uint32_t sampleCount = 0;
float dcSum = 0.0f;

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

        ch.sumCos = 0.0f;
        ch.sumSin = 0.0f;
    }
}

// ----------------------------
// Report completed window
// ----------------------------

void reportWindow() {
    float dcRaw = dcSum / WINDOW_SIZE;
    float dc = dcRaw * ADC_SCALE;

    float amplitude[3];

    for (int i = 0; i < 3; i++) {
        Channel &ch = channels[i];

        // Remove DC leakage from correlation
        float I = ch.sumI - dcRaw * ch.sumCos;
        float Q = ch.sumQ - dcRaw * ch.sumSin;

        amplitude[i] =
            (2.0f / WINDOW_SIZE) *
            sqrtf(I * I + Q * Q) *
            ADC_SCALE;
    }

    if (!Serial.isConnected() ||
        Serial.availableForWrite() < 100) {
        return;
    }

    Serial.print(dc, 6);

    for (int i = 0; i < 3; i++) {
        Serial.print(",");
        Serial.print(amplitude[i], 6);
    }

    Serial.print(",");
    Serial.print(droppedBuffers);

    Serial.print(",");
    Serial.println(invalidWindows);
}

// ----------------------------
// Process one ADC sample
// ----------------------------

void processSample(int16_t raw) {
    // Single-ended readings can be slightly
    // negative due to ADC offset.
    float sample = (raw < 0) ? 0.0f : (float)raw;

    dcSum += sample;

    for (int i = 0; i < 3; i++) {
        Channel &ch = channels[i];

        ch.sumI += sample * ch.cosPhase;
        ch.sumQ += sample * ch.sinPhase;

        ch.sumCos += ch.cosPhase;
        ch.sumSin += ch.sinPhase;

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

    if (sampleCount == WINDOW_SIZE) {
        reportWindow();
        resetWindow();
    }
}

// ----------------------------
// Process completed DMA buffer
// ----------------------------

void processBuffer(uint32_t index) {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        processSample(adcBuffers[index][i]);
    }
}

// ----------------------------
// SAADC interrupt handler
// ----------------------------

void saadcInterrupt() {
    // One DMA buffer has completed.
    if (NRF_SAADC->EVENTS_END) {
        NRF_SAADC->EVENTS_END = 0;

        completedBuffers++;
    }

    // A new DMA buffer has started.
    // Prepare the buffer after the active one.
    if (NRF_SAADC->EVENTS_STARTED) {
        NRF_SAADC->EVENTS_STARTED = 0;

        uint32_t nextIndex =
            (completedBuffers + 1) % BUFFER_COUNT;

        NRF_SAADC->RESULT.PTR =
            (uint32_t)adcBuffers[nextIndex];

        NRF_SAADC->RESULT.MAXCNT = BUFFER_SIZE;
    }
}

// ----------------------------
// Configure SAADC hardware
// ----------------------------

bool startADC() {
    // Do not use analogRead() after this point.

    NRF_SAADC->ENABLE = 0;

    // Disable all channels
    for (int i = 0; i < 8; i++) {
        NRF_SAADC->CH[i].PSELP = 0;
        NRF_SAADC->CH[i].PSELN = 0;
    }

    // A1 on Argon = nRF52840 AIN2
    NRF_SAADC->CH[0].PSELP =
        SAADC_CH_PSELP_PSELP_AnalogInput2;

    NRF_SAADC->CH[0].PSELN = 0;

    // Match Particle's nominal 0-3.3V
    // single-ended ADC configuration.
    NRF_SAADC->CH[0].CONFIG =
        (SAADC_CH_CONFIG_GAIN_Gain1_4
            << SAADC_CH_CONFIG_GAIN_Pos) |
        (SAADC_CH_CONFIG_REFSEL_VDD1_4
            << SAADC_CH_CONFIG_REFSEL_Pos) |
        (SAADC_CH_CONFIG_TACQ_10us
            << SAADC_CH_CONFIG_TACQ_Pos) |
        (SAADC_CH_CONFIG_MODE_SE
            << SAADC_CH_CONFIG_MODE_Pos);

    // 12-bit resolution, no oversampling
    NRF_SAADC->RESOLUTION =
        SAADC_RESOLUTION_VAL_12bit;

    NRF_SAADC->OVERSAMPLE = 0;

    // Internal timer:
    // 16 MHz / 800 = 20,000 Hz
    const uint32_t cc = 16000000UL / SAMPLE_RATE;

    NRF_SAADC->SAMPLERATE =
        (SAADC_SAMPLERATE_MODE_Timers
            << SAADC_SAMPLERATE_MODE_Pos) |
        (cc << SAADC_SAMPLERATE_CC_Pos);

    // Initial DMA buffer
    NRF_SAADC->RESULT.PTR =
        (uint32_t)adcBuffers[0];

    NRF_SAADC->RESULT.MAXCNT = BUFFER_SIZE;

    // Clear existing interrupts/events
    NRF_SAADC->INTENCLR = 0xFFFFFFFFUL;

    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->EVENTS_STARTED = 0;

    // Attach direct interrupt handler
    if (!attachInterruptDirect(
            SAADC_IRQn, saadcInterrupt, false)) {
        return false;
    }

    // Allocate a PPI channel managed by
    // the existing Nordic SDK.
    ret_code_t err =
        nrfx_ppi_channel_alloc(&ppiChannel);

    if (err != 0) {
        return false;
    }

    // Automatically restart the SAADC
    // when a DMA buffer is full.
    err = nrfx_ppi_channel_assign(
        ppiChannel,
        (uint32_t)&NRF_SAADC->EVENTS_END,
        (uint32_t)&NRF_SAADC->TASKS_START
    );

    if (err != 0) {
        return false;
    }

    err = nrfx_ppi_channel_enable(ppiChannel);

    if (err != 0) {
        return false;
    }

    // Enable END and STARTED interrupts
    NRF_SAADC->INTENSET =
        SAADC_INTENSET_END_Msk |
        SAADC_INTENSET_STARTED_Msk;

    // Low interrupt priority compatible
    // with the Nordic SoftDevice.
    NVIC_SetPriority(SAADC_IRQn, 6);
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    NVIC_EnableIRQ(SAADC_IRQn);

    // Enable ADC
    NRF_SAADC->ENABLE =
        SAADC_ENABLE_ENABLE_Enabled;

    // Begin acquisition
    NRF_SAADC->TASKS_START = 1;

    return true;
}

// ----------------------------
// Setup
// ----------------------------

void setup() {
    Serial.begin(115200);

    // Configure LEDs
    for (int i = 0; i < 3; i++) {
        pinMode(LED_PINS[i], OUTPUT);

        analogWrite(
            LED_PINS[i],
            128,
            LED_FREQS[i]
        );

        float angle =
            TWO_PI_F * LED_FREQS[i] / SAMPLE_RATE;

        channels[i].cosStep = cosf(angle);
        channels[i].sinStep = sinf(angle);
    }

    resetWindow();

    adcRunning = startADC();
}

// ----------------------------
// Main loop
// ----------------------------

void loop() {
    if (Serial.isConnected()) {
        if (!headerPrinted) {
            if (adcRunning) {
                Serial.println(
                    "dc_v,red_v,ir805_v,ir940_v,"
                    "dropped_buffers,invalid_windows"
                );
            } else {
                Serial.println("ERROR: ADC initialization failed");
            }

            headerPrinted = true;
        }
    } else {
        headerPrinted = false;
    }

    if (!adcRunning) {
        return;
    }

    uint32_t available = completedBuffers;

    if (available == processedBuffers) {
        return;
    }

    // Avoid reading buffers that DMA is
    // about to reuse.
    uint32_t backlog = available - processedBuffers;

    if (backlog > BUFFER_COUNT - 2) {
        uint32_t skipped = backlog - 1;

        droppedBuffers += skipped;
        invalidWindows++;

        processedBuffers = available - 1;

        resetWindow();
    }

    uint32_t index =
        processedBuffers % BUFFER_COUNT;

    processBuffer(index);

    processedBuffers++;
}
