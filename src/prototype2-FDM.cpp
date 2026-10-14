
/*
 * Three-Wavelength Oximeter Prototype
 *
 * Uses hardware PWM to flash three LEDs at
 * independent frequencies for Frequency
 * Division Multiplexing (FDM).
 *
 * Target: Particle Argon / Boron
 */

#include "Particle.h"

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

// LED pins (three different PWM timer groups)
#define RED_LED   D8
#define GREEN_LED A4
#define BLUE_LED  A0

// LED modulation frequencies (Hz)
const uint32_t RED_FREQ   = 7;
const uint32_t GREEN_FREQ = 11;
const uint32_t BLUE_FREQ  = 17;

// 50% duty cycle (8-bit PWM)
const uint8_t DUTY_CYCLE = 128;

SerialLogHandler logHandler(LOG_LEVEL_INFO);

void setup() {
    // Configure LED pins as outputs
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    // Generate independent PWM signals
    analogWrite(RED_LED, DUTY_CYCLE, RED_FREQ);
    analogWrite(GREEN_LED, DUTY_CYCLE, GREEN_FREQ);
    analogWrite(BLUE_LED, DUTY_CYCLE, BLUE_FREQ);

    Log.info("Three LED PWM signals started");
    Log.info("Red: %lu Hz", (unsigned long)RED_FREQ);
    Log.info("Green: %lu Hz", (unsigned long)GREEN_FREQ);
    Log.info("Blue: %lu Hz", (unsigned long)BLUE_FREQ);
}

void loop() {
    // Hardware PWM runs automatically.
    // Future photodiode sampling code goes here.
}