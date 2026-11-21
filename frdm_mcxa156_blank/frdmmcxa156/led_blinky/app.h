/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

#include "fsl_gpio.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

/* NeoPixel (WS2812B) strip data pin: P2_12, header J2 pin 12. */
#define NEOPIXEL_PORT  PORT2
#define NEOPIXEL_GPIO  GPIO2
#define NEOPIXEL_PIN   12U
/* Number of pixels on the strip (sending more than you have is harmless). */
#define NEOPIXEL_COUNT 30U
/* Global brightness 0-255. Keep it modest: each pixel draws up to 60 mA at full white. */
#define NEOPIXEL_BRIGHTNESS 80U
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */
