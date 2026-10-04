#pragma once

#include <Arduino.h>

/**
 * @file pins.h
 * @brief Master Pin Definitions for MicroLink on ESP32-C6 DevKitC-1
 *
 * Hardware Notes:
 * - ESP32-C6: Single-core RISC-V 160 MHz, 320 KB SRAM, 8 MB Flash.
 * - Single I2S peripheral (I2S_NUM_0).
 * - ADC1 channels available only on GPIO 0 through 6.
 *
 * Strapping and reserved pins avoided:
 * - GPIO 4, 5, 8, 9, 15: Boot strapping pins
 * - GPIO 12, 13: USB Serial/JTAG
 * - GPIO 16, 17: UART0 (Serial programming / console fallback)
 * - GPIO 24..30: Integrated SPI Flash bus
 */

// =============================================================================
// I2S Digital Audio Output (PCM5102A DAC)
// =============================================================================
// PCM5102A hardware strap configuration:
//   SCK  -> GND  (PCM5102A generates internal SCK from BCK via internal PLL)
//   FLT  -> GND  (Normal latency filter / FIR filter)
//   DMP  -> GND  (De-emphasis off)
//   FMT  -> GND  (Standard I2S data format)
//   XSMT -> 3V3  (Soft mute disabled / active audio output)
constexpr int PIN_I2S_BCK       = 18; // Bit Clock (BCK)
constexpr int PIN_I2S_WS        = 19; // Word Select / Left-Right Clock (LCK)
constexpr int PIN_I2S_DOUT      = 20; // Data Out -> DIN on PCM5102A

// =============================================================================
// Analog Audio Input (Microphone)
// =============================================================================
// KY-038 Microphone Module:
//   Connect Analog Output (AO) to ADC1_CH1 (GPIO 1).
//   Note: Keep behind AudioIn abstraction to allow future swap with I2S MEMS mic.
constexpr int PIN_MIC_AO        = 1;  // ADC1 Channel 1

// =============================================================================
// Status Indicators
// =============================================================================
// Active HIGH (standard LED with current limiting resistor to GND)
constexpr int PIN_LED_GREEN     = 10; // WiFi connected + EchoLink registered
constexpr int PIN_LED_RED       = 11; // Transmitter active (PTT / TX engaged)
constexpr int PIN_LED_RX        = 22; // RX active (receiving audio from remote station)

// =============================================================================
// Controls & Inputs
// =============================================================================
// Push Button for PTT (Push-To-Talk) / Manual Control:
// Active LOW, requires internal pull-up enabled (INPUT_PULLUP)
constexpr int PIN_BUTTON_PTT    = 2;

// Potentiometer for VOX sensitivity (ADC1 Channel 3 on GPIO 3)
constexpr int PIN_POT_VOX_SENS  = 3;

// VOX mode switch (Slide switch / Toggle switch, Active LOW with INPUT_PULLUP on GPIO 23)
// LOW = VOX mode, HIGH = PTT mode
constexpr int PIN_SWITCH_VOX    = 23;

// =============================================================================
// Optional I2C Display (SSD1306 OLED 128x64)
// =============================================================================
#if defined(ENABLE_OLED) && (ENABLE_OLED == 1)
constexpr int PIN_OLED_SDA      = 6;  // I2C SDA
constexpr int PIN_OLED_SCL      = 7;  // I2C SCL
#endif
