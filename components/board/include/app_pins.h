/*
 * Single source of truth for every GPIO in the design.
 *
 * ESP32-WROOM-32 constraints honoured here:
 *   - GPIO6..11 are wired to the SPI flash and must never be used.
 *   - GPIO0, 2, 12, 15 are strapping pins; avoided for anything driven at boot.
 *     (GPIO12/MTDI is the nastiest: held high at reset it selects a 1.8 V flash
 *     voltage and the module will not boot.)
 *   - GPIO34..39 are input-only. That is exactly what the I2S input needs, so
 *     the three incoming clock/data lines are placed there and the more
 *     valuable bidirectional pins are left for everything else.
 *   - GPIO1/GPIO3 stay on UART0 for the serial console. We can afford this only
 *     because the DAC needs no MCLK -- MCLK on ESP32 is restricted to GPIO0/1/3.
 */
#pragma once

#include "driver/gpio.h"

/* ------------------------------------------------------------------ */
/* I2S input -- ESP32 is the SLAVE, the external source drives the clocks */
/* ------------------------------------------------------------------ */
#define PIN_I2S_IN_BCLK     GPIO_NUM_34   /* input-only */
#define PIN_I2S_IN_WS       GPIO_NUM_35   /* input-only; also fanned to PCNT   */
#define PIN_I2S_IN_DATA     GPIO_NUM_36   /* input-only (SENSOR_VP)            */

/* ------------------------------------------------------------------ */
/* I2S output -- ESP32 is the MASTER, driving the DAC                  */
/* ------------------------------------------------------------------ */
#define PIN_I2S_OUT_BCLK    GPIO_NUM_26
#define PIN_I2S_OUT_WS      GPIO_NUM_25
#define PIN_I2S_OUT_DATA    GPIO_NUM_22

/* ------------------------------------------------------------------ */
/* TFT panel on VSPI (SPI3)                                            */
/* ------------------------------------------------------------------ */
#define PIN_TFT_SCLK        GPIO_NUM_18
#define PIN_TFT_MOSI        GPIO_NUM_23
#define PIN_TFT_CS          GPIO_NUM_5
#define PIN_TFT_DC          GPIO_NUM_21
#define PIN_TFT_RST         GPIO_NUM_19
#define PIN_TFT_BACKLIGHT   GPIO_NUM_4

/* ------------------------------------------------------------------ */
/* EC11 rotary encoder + back button                                   */
/* ------------------------------------------------------------------ */
#define PIN_ENC_A           GPIO_NUM_32
#define PIN_ENC_B           GPIO_NUM_33
#define PIN_ENC_PUSH        GPIO_NUM_27   /* the encoder's own switch = SELECT */
#define PIN_BTN_BACK        GPIO_NUM_14
